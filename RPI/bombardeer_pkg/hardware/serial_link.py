"""
Robust ESP32 UART serial worker (Core 3).
- Strictly monotonic timers; drains non-droppable TurretCommandBus.
- Preserves ESP32 home across restarts (DTR/RTS cleared pre-open, HUPCL disabled).
- Auto-recovers on consecutive TX failures or prolonged RX telemetry silence.
"""

import logging
import os
import time

import serial

from bombardeer_pkg.config import BombardeerConfig
from bombardeer_pkg.state import SystemState
from bombardeer_pkg.utils import set_os_thread_name

try:
    import termios
except ImportError:
    termios = None

log = logging.getLogger("Bombardeer")


def _disable_hupcl(fd: int):
    if termios is None:
        return
    try:
        attrs = termios.tcgetattr(fd)
        attrs[2] &= ~termios.HUPCL
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
    except Exception as exc:
        log.debug(f"[SERIAL] Could not clear HUPCL: {exc}")


def find_working_serial_port(ports):
    for port in ports:
        if os.path.exists(port) and os.access(port, os.R_OK | os.W_OK):
            return port
    return None


def _safe_close(ser):
    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass


def turret_serial_worker(cfg: BombardeerConfig, state: SystemState):
    ser_cfg = cfg.serial
    ctrl = cfg.control
    bus = state.command_bus

    ser = None
    reconnect_delay = 2.0
    next_reconnect_mono = 0.0
    last_missing_log_mono = 0.0

    trigger_state = 0
    trigger_start_mono = 0.0
    active_pulse_limit = ctrl.max_trigger_duration_sec
    cooldown_until_mono = 0.0

    rx_buf = bytearray()
    last_telemetry_rx_mono = 0.0
    consecutive_tx_errors = 0

    set_os_thread_name("TurretSerial", cpu_cores=[3])

    while not state.shutdown_event.is_set():
        mono_now = time.monotonic()

        if ser is None or not ser.is_open:
            state.update_health(serial_connected=False)
            bus.clear_motion()
            if mono_now >= next_reconnect_mono:
                active_port = find_working_serial_port(ser_cfg.ports)
                if active_port:
                    try:
                        ser = serial.Serial()
                        ser.port = active_port
                        ser.baudrate = ser_cfg.baud_rate
                        ser.timeout = 0.01
                        ser.write_timeout = 0.1
                        ser.rtscts = False
                        ser.dsrdtr = False
                        ser.dtr = False
                        ser.rts = False
                        ser.open()
                        _disable_hupcl(ser.fileno())

                        rx_buf.clear()
                        consecutive_tx_errors = 0
                        last_telemetry_rx_mono = mono_now

                        time.sleep(0.05)
                        ser.reset_input_buffer()
                        ser.reset_output_buffer()

                        state.update_health(serial_connected=True)
                        log.info(f"[SERIAL] Connected to {active_port} @ {ser_cfg.baud_rate} baud (no-reset).")
                        state.set_alert(f"Connected: {active_port}", duration=2.0)
                    except Exception as exc:
                        _safe_close(ser)
                        ser = None
                        next_reconnect_mono = mono_now + reconnect_delay
                        state.set_alert(f"Serial Open Error: {exc}", duration=2.0)
                        log.error(f"[SERIAL] Failed connecting to {active_port}: {exc}")
                else:
                    if mono_now - last_missing_log_mono >= 4.0:
                        last_missing_log_mono = mono_now
                        log.warning(f"[SERIAL] No ESP32 found on {ser_cfg.ports}.")
                        state.set_alert("ESP32 DISCONNECTED", duration=3.0)
                    next_reconnect_mono = mono_now + reconnect_delay

        if ser and ser.is_open:
            rx_age = mono_now - last_telemetry_rx_mono
            state.update_health(serial_connected=(rx_age <= ser_cfg.heartbeat_timeout_sec))
            if rx_age > ser_cfg.reconnect_no_rx_sec:
                log.warning(f"[SERIAL] No RX telemetry for {rx_age:.1f}s; reopening port.")
                _safe_close(ser)
                ser = None
                state.update_health(serial_connected=False)
                next_reconnect_mono = mono_now + reconnect_delay
                continue

        commands = bus.drain()
        if ser and ser.is_open:
            try:
                for item in commands:
                    cmd = item["cmd"]
                    if cmd == "HALT":
                        ser.write(b"X\n")
                        state.inc_health("serial_tx_count", 1)
                    elif cmd == "TRIGGER":
                        req_state = item["state"]
                        req_mono = item.get("timestamp_mono", mono_now)
                        if req_state == 0:
                            if trigger_state == 1:
                                trigger_state = 0
                                state.set_firing_state(False)
                                cooldown_until_mono = mono_now + ctrl.trigger_cooldown_sec
                                ser.write(b"T 0\n")
                                state.inc_health("serial_tx_count", 1)
                        elif req_state == 1 and (mono_now - req_mono <= 0.100):
                            if trigger_state == 0 and mono_now >= cooldown_until_mono:
                                trigger_state = 1
                                trigger_start_mono = mono_now
                                custom_pulse = item.get("pulse_len")
                                active_pulse_limit = custom_pulse if custom_pulse else ctrl.max_trigger_duration_sec
                                state.set_firing_state(True)
                                ser.write(b"T 1\n")
                                state.inc_health("serial_tx_count", 1)
                                log.info(f"[SERIAL] Solenoid FIRED (limit {active_pulse_limit * 1000:.0f} ms)")
                    elif cmd == "SETPOINT":
                        ser.write(f"P {int(item['pan'])} {int(item['tilt'])}\n".encode("ascii"))
                        state.inc_health("serial_tx_count", 1)
                    elif cmd == "VELOCITY":
                        ser.write(f"V {int(item['pan_s'])} {int(item['tilt_s'])}\n".encode("ascii"))
                        state.inc_health("serial_tx_count", 1)

                if trigger_state == 1 and (mono_now - trigger_start_mono >= active_pulse_limit):
                    trigger_state = 0
                    state.set_firing_state(False)
                    cooldown_until_mono = mono_now + ctrl.trigger_cooldown_sec
                    ser.write(b"T 0\n")
                    state.inc_health("serial_tx_count", 1)

                consecutive_tx_errors = 0
            except (serial.SerialException, OSError) as exc:
                consecutive_tx_errors += 1
                log.warning(f"[SERIAL TX FAULT #{consecutive_tx_errors}] {exc}")
                if consecutive_tx_errors >= ser_cfg.max_consecutive_tx_errors:
                    _safe_close(ser)
                    ser = None
                    state.update_health(serial_connected=False)
                    next_reconnect_mono = mono_now + reconnect_delay
                    continue

        if ser and ser.is_open:
            try:
                avail = ser.in_waiting
                if avail > 0:
                    rx_buf.extend(ser.read(avail))
                    while True:
                        nl = rx_buf.find(b"\n")
                        if nl == -1:
                            break
                        line = rx_buf[:nl].strip()
                        del rx_buf[:nl + 1]
                        if not line:
                            continue
                        if not line.startswith(b"S "):
                            log.warning(f"[ESP32] {line.decode('ascii', errors='replace')}")
                            continue
                        rx_mono = time.monotonic()
                        parts = line.split()
                        try:
                            if len(parts) >= 6:
                                p_mrad, t_mrad, moving = int(parts[2]), int(parts[3]), (parts[5] == b"1")
                            elif len(parts) >= 5:
                                p_mrad, t_mrad, moving = int(parts[1]), int(parts[2]), (parts[4] == b"1")
                            else:
                                continue
                            last_telemetry_rx_mono = rx_mono
                            state.update_turret_telemetry(p_mrad, t_mrad, moving, rx_mono)
                            state.inc_health("serial_rx_count", 1)
                        except ValueError:
                            continue
            except (serial.SerialException, OSError) as exc:
                log.warning(f"[SERIAL RX FAULT] {exc}")
                _safe_close(ser)
                ser = None
                state.update_health(serial_connected=False)
                next_reconnect_mono = mono_now + reconnect_delay

        state.shutdown_event.wait(timeout=0.002)

    if ser and ser.is_open:
        try:
            ser.write(b"T 0\nX\n")
            ser.flush()
        except Exception:
            pass
        _safe_close(ser)
    log.info("[SERIAL] TurretSerial shut down cleanly.")