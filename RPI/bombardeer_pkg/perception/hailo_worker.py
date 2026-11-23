"""
Spawn-safe isolated Hailo-8 NPU subprocess (Core 2).
Uses Pipe token IPC (full ARM64 memory barriers) and a 2-buffer prep/infer
pipeline so CPU slicing/resizing overlaps with Hailo-8 PCIe DMA.
"""

import logging
import os
import queue
import signal
import threading
import time
import traceback
from multiprocessing import resource_tracker, shared_memory

import cv2
import numpy as np

from bombardeer_pkg.config import CameraConfig, OpticsConfig, WildlifeConfig
from bombardeer_pkg.perception.mdv6_decoder import MDV6Decoder1280
from bombardeer_pkg.perception.tiling import fill_tensor_from_slots
from bombardeer_pkg.utils import configure_logging, set_os_thread_name

log = logging.getLogger("Bombardeer")


def hailo_worker_main(
    model_path: str,
    shm_name: str,
    req_rx,
    ack_tx,
    res_tx,
    avail_flag,
    stop_flag,
    cam: CameraConfig,
    optics: OpticsConfig,
    wl: WildlifeConfig,
):
    """Entry point launched via mp.get_context('spawn') with only picklable args."""
    configure_logging()
    cv2.setNumThreads(1)
    cv2.ocl.setUseOpenCL(False)
    set_os_thread_name("HailoNPUProc", cpu_cores=[2])

    def _sig_handler(signum, frame):
        stop_flag.value = 1

    signal.signal(signal.SIGTERM, _sig_handler)
    signal.signal(signal.SIGINT, _sig_handler)

    from hailo_platform import (
        ConfigureParams,
        FormatType,
        HEF,
        HailoStreamInterface,
        InferVStreams,
        InputVStreamParams,
        OutputVStreamParams,
        VDevice,
    )

    shm = None
    try:
        shm = shared_memory.SharedMemory(name=shm_name)
        resource_tracker.unregister(shm._name, "shared_memory")

        hef = HEF(model_path)
        decoder = MDV6Decoder1280(cam=cam, optics=optics, wl=wl, hef=hef)

        params = VDevice.create_params()
        with VDevice(params) as target:
            cfg_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
            network_group = target.configure(hef, cfg_params)[0]
            ng_params = network_group.create_params()

            in_vparams = InputVStreamParams.make(network_group, quantized=True, format_type=FormatType.UINT8)
            out_fmt = FormatType.UINT8 if decoder.use_uint8_fastpath else FormatType.FLOAT32
            out_vparams = OutputVStreamParams.make(
                network_group, quantized=decoder.use_uint8_fastpath, format_type=out_fmt
            )
            input_name = hef.get_input_vstream_infos()[0].name

            tensor_pool = [
                np.full((1, wl.infer_size, wl.infer_size, 3), wl.stack_divider_gray, dtype=np.uint8),
                np.full((1, wl.infer_size, wl.infer_size, 3), wl.stack_divider_gray, dtype=np.uint8),
            ]

            with network_group.activate(ng_params):
                with InferVStreams(network_group, in_vparams, out_vparams) as pipeline:
                    avail_flag.value = 1
                    log.info(
                        f"[HAILO NPU] Spawned PID:{os.getpid()} active "
                        f"({'UINT8' if decoder.use_uint8_fastpath else 'FP32'}, "
                        f"rgb_direct={cam.main_stream_is_rgb}): {model_path}"
                    )

                    last_fps_t = time.monotonic()
                    sweeps = 0
                    cur_fps = 0.0

                    while stop_flag.value == 0 and os.getppid() != 1:
                        if not req_rx.poll(0.05):
                            continue
                        try:
                            req = req_rx.recv()
                        except (EOFError, OSError):
                            break
                        if req is None:
                            break

                        t0 = time.perf_counter()
                        (
                            cap_mono,
                            sw,
                            sh,
                            roi_x0,
                            roi_y0,
                            roi_w,
                            roi_h,
                            mode_tag,
                            pass_plan,
                            was_tracking,
                            was_force_scan,
                        ) = req

                        if roi_w <= 0 or roi_h <= 0 or (roi_w * roi_h * 3) > shm.size:
                            ack_tx.send(True)
                            continue

                        roi_view = np.ndarray((roi_h, roi_w, 3), dtype=np.uint8, buffer=shm.buf)
                        veto_acc = {"boxes": [], "scores": [], "meta": []}
                        anim_acc = {"boxes": [], "scores": [], "probs": []}

                        if len(pass_plan) <= 1:
                            if pass_plan:
                                fill_tensor_from_slots(
                                    roi_view,
                                    pass_plan[0],
                                    tensor_pool[0][0],
                                    roi_x0=roi_x0,
                                    roi_y0=roi_y0,
                                    src_is_rgb=cam.main_stream_is_rgb,
                                    divider_gray=wl.stack_divider_gray,
                                    tile_w=wl.stack_tile_w,
                                    tile_h=wl.stack_tile_h,
                                    slot1_y=wl.stack_slot1_y,
                                )
                            ack_tx.send(True)
                            if pass_plan:
                                raw_out = pipeline.infer({input_name: tensor_pool[0]})
                                decoder.decode_pass(raw_out, pass_plan[0], sw, sh, veto_acc, anim_acc)
                        else:
                            prep_q = queue.Queue(maxsize=1)

                            def _prep_passes():
                                for idx, slots in enumerate(pass_plan):
                                    buf_idx = idx & 1
                                    fill_tensor_from_slots(
                                        roi_view,
                                        slots,
                                        tensor_pool[buf_idx][0],
                                        roi_x0=roi_x0,
                                        roi_y0=roi_y0,
                                        src_is_rgb=cam.main_stream_is_rgb,
                                        divider_gray=wl.stack_divider_gray,
                                        tile_w=wl.stack_tile_w,
                                        tile_h=wl.stack_tile_h,
                                        slot1_y=wl.stack_slot1_y,
                                    )
                                    if idx == len(pass_plan) - 1:
                                        ack_tx.send(True)
                                    prep_q.put((buf_idx, slots))
                                prep_q.put(None)

                            prep_t = threading.Thread(target=_prep_passes, daemon=True)
                            prep_t.start()
                            while True:
                                item = prep_q.get()
                                if item is None:
                                    break
                                buf_idx, slots = item
                                raw_out = pipeline.infer({input_name: tensor_pool[buf_idx]})
                                decoder.decode_pass(raw_out, slots, sw, sh, veto_acc, anim_acc)
                            prep_t.join()

                        dets, debug_info = decoder.finalize_sweep(veto_acc, anim_acc)
                        dt_ms = (time.perf_counter() - t0) * 1000.0

                        sweeps += 1
                        mono_now = time.monotonic()
                        elapsed = mono_now - last_fps_t
                        if elapsed >= 1.0:
                            cur_fps = sweeps / elapsed
                            sweeps = 0
                            last_fps_t = mono_now

                        try:
                            res_tx.send(
                                (dets, debug_info, cap_mono, dt_ms, cur_fps, mode_tag, was_tracking, was_force_scan)
                            )
                        except (BrokenPipeError, EOFError, OSError):
                            break

    except Exception as exc:
        if stop_flag.value == 0:
            log.error(f"[HAILO SUBPROCESS FATAL] {exc}\n{traceback.format_exc()}")
    finally:
        avail_flag.value = 0
        if shm is not None:
            try:
                shm.close()
            except Exception:
                pass
        log.info(f"[HAILO NPU] PID:{os.getpid()} shut down cleanly.")