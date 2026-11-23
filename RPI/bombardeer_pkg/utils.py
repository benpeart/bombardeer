"""OS helpers: logging, CPU pinning, systemd watchdog, single-instance lock, audit snapshots."""

import ctypes
import datetime
try:
    import fcntl
except ImportError:
    fcntl = None
import glob
import logging
import os
import socket
import time

import cv2

log = logging.getLogger("Bombardeer")


def configure_logging():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )


def set_os_thread_name(name: str, cpu_cores=None):
    """Sets Linux thread name (prctl PR_SET_NAME) and optional CPU core affinity."""
    try:
        libc = ctypes.CDLL("libc.so.6")
        libc.prctl(15, name[:15].encode("utf-8"), 0, 0, 0)
    except Exception:
        pass
    if cpu_cores is not None:
        try:
            os.sched_setaffinity(0, set(cpu_cores))
        except Exception:
            pass


def sd_notify(message: str) -> bool:
    """Zero-dependency systemd sd_notify sender (e.g. 'READY=1', 'WATCHDOG=1')."""
    addr = os.environ.get("NOTIFY_SOCKET")
    if not addr:
        return False
    if addr.startswith("@"):
        addr = "\0" + addr[1:]
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
            sock.sendto(message.encode("utf-8"), addr)
        return True
    except Exception:
        return False


def read_soc_temp_c():
    """Reads Raspberry Pi SoC temperature in Celsius, or None if unavailable."""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r", encoding="utf-8") as fh:
            return round(int(fh.read().strip()) / 1000.0, 1)
    except Exception:
        return None


class InstanceLock:
    """Exclusive fcntl PID lockfile to prevent duplicate Gunicorn workers or processes."""

    def __init__(self, path: str):
        self.path = path
        self._fd = None

    def acquire(self) -> bool:
        try:
            self._fd = os.open(self.path, os.O_CREAT | os.O_RDWR, 0o644)
            fcntl.flock(self._fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            os.ftruncate(self._fd, 0)
            os.write(self._fd, f"{os.getpid()}\n".encode("ascii"))
            return True
        except OSError:
            if self._fd is not None:
                try:
                    os.close(self._fd)
                except OSError:
                    pass
                self._fd = None
            return False

    def release(self):
        if self._fd is not None:
            try:
                fcntl.flock(self._fd, fcntl.LOCK_UN)
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None


class RateLimitedLogger:
    """Prevents log floods on recurring hardware faults while never silently swallowing errors."""

    def __init__(self, interval_sec: float = 5.0):
        self._interval = interval_sec
        self._last_logged = {}

    def warning(self, key: str, message: str):
        now = time.monotonic()
        if now - self._last_logged.get(key, -1e9) >= self._interval:
            self._last_logged[key] = now
            log.warning(message)

    def error(self, key: str, message: str):
        now = time.monotonic()
        if now - self._last_logged.get(key, -1e9) >= self._interval:
            self._last_logged[key] = now
            log.error(message)


def ensure_storage_dirs(recordings_dir: str, debug_snapshot_dir: str):
    try:
        os.makedirs(recordings_dir, exist_ok=True)
        os.makedirs(debug_snapshot_dir, exist_ok=True)
        test_touch = os.path.join(recordings_dir, ".write_test")
        with open(test_touch, "w", encoding="utf-8") as fh:
            fh.write("ok")
        os.remove(test_touch)
        log.info(f"[STORAGE] Storage directory verified writable: {recordings_dir}")
    except Exception as exc:
        log.error(f"[STORAGE ERROR] Cannot write to storage path {recordings_dir}: {exc}")


def save_audit_snapshot(frame_bgr, tag: str, dets: list, debug_info: dict, debug_dir: str, max_snapshots: int):
    """Saves an annotated 1280x720 JPEG with Animal (green), Rejected (orange), and Veto (red) boxes."""
    try:
        vis = frame_bgr.copy()
        for v in debug_info.get("veto_boxes", []):
            x1, y1, x2, y2 = v["box"]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(
                vis, f"VETO:{v['class']} {v['score']:.2f}",
                (x1, max(y1 - 6, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2,
            )
        for r in debug_info.get("rejected_animals", []):
            x1, y1, x2, y2 = r["box"]
            pa, pp, pv = r["probs"]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 165, 255), 2)
            cv2.putText(
                vis, f"REJ:{r['reason']} A:{pa:.2f} P:{pp:.2f} V:{pv:.2f}",
                (x1, max(y1 - 6, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2,
            )
        for d in dets:
            x1, y1, x2, y2 = d["box"]
            pa, pp, pv = d.get("probs", (d["score"], 0.0, 0.0))
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                vis, f"ACQ A:{pa:.2f} P:{pp:.2f} V:{pv:.2f}",
                (x1, max(y1 - 6, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
            )
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        out_path = os.path.join(debug_dir, f"{tag}_{ts}.jpg")
        cv2.imwrite(out_path, vis, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        log.info(f"[AUDIT] Saved {tag} snapshot -> {out_path}")

        snaps = glob.glob(os.path.join(debug_dir, "*.jpg"))
        if len(snaps) > max_snapshots:
            snaps.sort(key=os.path.getmtime)
            for old_snap in snaps[:-max_snapshots]:
                try:
                    os.remove(old_snap)
                except OSError:
                    pass
    except Exception as exc:
        log.warning(f"[AUDIT] Failed to save snapshot: {exc}")