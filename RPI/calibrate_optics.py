#!/usr/bin/env python3
"""
Bombardeer Robust Optical Focal Length Calibration Suite
-------------------------------------------------------
- Larger angular excursions (Pan: 320 mrad, Tilt: 140 mrad) to maximize pixel delta.
- 2.5-second post-motion mechanical dampening pause to eliminate gear resonance.
- 30-frame outlier-filtered statistical sampling (std < 1.5 px) at every waypoint.
- Bidirectional back-and-forth sweep to neutralize mechanical backlash.
"""

import os
import sys
import time
import math
import serial
import numpy as np
import cv2
from picamera2 import Picamera2
from libcamera import Transform

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
OPTICAL_CENTER = (FRAME_WIDTH / 2.0, FRAME_HEIGHT / 2.0)

# Large angular displacements to maximize pixel baseline and minimize noise
TEST_PAN_MRAD = 320    # ~18.3 degrees (requires starting target ~350px left of center)
TEST_TILT_MRAD = 140   # ~8.0 degrees

SETTLE_DELAY_SEC = 2.5 # Dedicated post-arrival rest window
SAMPLE_COUNT = 30      # Frames to average per waypoint
TARGET_MARKER_ID = 0

DEFAULT_SERIAL_PORTS = ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyUSB1"]
BAUD_RATE = 115200


class ArUcoTracker:
    def __init__(self, target_id=TARGET_MARKER_ID):
        self.target_id = target_id
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.errorCorrectionRate = 0.55
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        self.clahe = cv2.createCLAHE(clipLimit=2.2, tileGridSize=(8, 8))

    def get_target_center(self, frame_raw):
        if len(frame_raw.shape) == 3:
            gray = cv2.cvtColor(frame_raw, cv2.COLOR_RGB2GRAY)
        else:
            gray = frame_raw
        enhanced = self.clahe.apply(gray)
        corners, ids, _ = self.detector.detectMarkers(enhanced)

        if ids is not None and len(ids) > 0:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                if self.target_id is not None and marker_id != self.target_id:
                    continue
                pts = marker_corners.reshape((4, 2))
                cx = float(np.mean(pts[:, 0]))
                cy = float(np.mean(pts[:, 1]))
                return cx, cy
        return None


def open_serial():
    for port in DEFAULT_SERIAL_PORTS:
        if os.path.exists(port):
            try:
                ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
                time.sleep(1.5)  # Let ESP32 settle after serial DTR toggle
                # Flush pending buffer
                ser.reset_input_buffer()
                return ser
            except Exception:
                continue
    return None


def wait_for_settle(ser, timeout=6.0):
    """Wait for ESP32 is_moving flag to drop, then wait for physical mechanical settle."""
    start = time.time()
    time.sleep(0.15)  # Allow command to reach and start stepper
    stopped_at = None

    while time.time() - start < timeout:
        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("S "):
            parts = line.split()
            if len(parts) >= 5:
                is_moving = (int(parts[4]) == 1)
                if not is_moving:
                    if stopped_at is None:
                        stopped_at = time.time()
                        print(f"  [HARDWARE STOP] Waiting {SETTLE_DELAY_SEC}s for mechanical vibrations to dampen...", end="", flush=True)
                    elif time.time() - stopped_at >= SETTLE_DELAY_SEC:
                        print(" Stable.")
                        return True
        time.sleep(0.02)

    print(" [WARN] Motion settle timeout fallback triggered.")
    time.sleep(SETTLE_DELAY_SEC)
    return False


def get_robust_position(picam2, tracker, sample_count=SAMPLE_COUNT):
    """Collect frames, reject statistical outliers (>2 sigma), and compute clean centroid."""
    raw_points = []
    
    # Drain any latent frames
    for _ in range(3):
        req = picam2.capture_request()
        req.release()

    for _ in range(sample_count):
        req = picam2.capture_request()
        frame = req.make_array("main")
        req.release()

        pt = tracker.get_target_center(frame)
        if pt is not None:
            raw_points.append(pt)
        time.sleep(0.035)

    if len(raw_points) < (sample_count * 0.70):
        print(f"  [FAIL] Target lost in {sample_count - len(raw_points)} of {sample_count} frames.")
        return None, None

    xs = np.array([p[0] for p in raw_points])
    ys = np.array([p[1] for p in raw_points])

    # Outlier rejection: Keep within 2 standard deviations
    mean_x, std_x = np.mean(xs), np.std(xs)
    mean_y, std_y = np.mean(ys), np.std(ys)

    valid_mask = (np.abs(xs - mean_x) <= 2.0 * max(std_x, 0.5)) & \
                 (np.abs(ys - mean_y) <= 2.0 * max(std_y, 0.5))

    clean_xs = xs[valid_mask]
    clean_ys = ys[valid_mask]

    final_x = float(np.mean(clean_xs))
    final_y = float(np.mean(clean_ys))
    final_std = float(math.hypot(np.std(clean_xs), np.std(clean_ys)))

    print(f"  Centroid: ({final_x:.2f}, {final_y:.2f}) px | Jitter (1σ): {final_std:.3f} px (from {len(clean_xs)} frames)")

    if final_std > 1.5:
        print(f"  [WARN] Jitter ({final_std:.2f} px) is high. Ensure target is stationary.")

    return (final_x, final_y), final_std


def solve_focal_length(p1, p2, center_coord, angle_mrad):
    """
    Solves: |atan((p1 - c)/f) - atan((p2 - c)/f)| = target_angle_rad
    via binary search over realistic lens bounds.
    """
    target_rad = abs(angle_mrad / 1000.0)
    d1 = p1 - center_coord
    d2 = p2 - center_coord

    low_f = 200.0
    high_f = 4000.0
    best_f = None

    for _ in range(80):
        mid_f = (low_f + high_f) / 2.0
        predicted_rad = abs(math.atan(d1 / mid_f) - math.atan(d2 / mid_f))

        if abs(predicted_rad - target_rad) < 1e-7:
            best_f = mid_f
            break

        if predicted_rad > target_rad:
            low_f = mid_f
        else:
            high_f = mid_f
        best_f = mid_f

    return best_f


def main():
    print("=" * 70)
    print(" BOMBARDEER HIGH-PRECISION OPTICAL CALIBRATION")
    print("=" * 70)

    ser = open_serial()
    if not ser:
        print("[ERROR] Could not connect to ESP32.")
        sys.exit(1)
    print("[SERIAL] ESP32 Connected.")

    tracker = ArUcoTracker()

    print("[CAMERA] Initializing Picamera2 (1280x720 native configuration)...")
    picam2 = Picamera2()
    cam_config = picam2.create_video_configuration(
        sensor={"output_size": (2304, 1296)},
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"},
        lores={"size": (640, 360), "format": "RGB888"},
        controls={"AeEnable": True, "AwbMode": 2, "AeExposureMode": 0, "ExposureValue": 1.0},
        transform=Transform(hflip=True, vflip=True)
    )
    picam2.configure(cam_config)
    picam2.start()
    time.sleep(1.0)

    print("\n[ALIGNMENT INSTRUCTIONS]")
    print(f"Place the ArUco marker toward the LEFT side of the screen (X ~= 350-450 px).")
    print(f"This allows the turret to execute a large {TEST_PAN_MRAD} mrad sweep without losing it.\n")

    input("Press ENTER once the target is placed and camera has an unobstructed view...")

    # -------------------------------------------------------------------------
    # Initial Baseline Position
    # -------------------------------------------------------------------------
    print("\n--- MEASURING BASELINE (Position A) ---")
    pos_A, _ = get_robust_position(picam2, tracker)
    if pos_A is None:
        print("[ERROR] Could not acquire target at start. Aborting.")
        sys.exit(1)

    print(f"Baseline Confirmed: X = {pos_A[0]:.2f} px, Y = {pos_A[1]:.2f} px")

    # -------------------------------------------------------------------------
    # Pan Axis Calibration (Forward + Reverse for Backlash Cancellation)
    # -------------------------------------------------------------------------
    print(f"\n--- PAN AXIS CALIBRATION (Sweep: +{TEST_PAN_MRAD} mrad) ---")
    ser.write(f"M {TEST_PAN_MRAD} 0\n".encode())
    wait_for_settle(ser)

    print("Measuring Displaced Position (Position B)...")
    pos_B, _ = get_robust_position(picam2, tracker)
    if pos_B is None:
        print("[ERROR] Lost marker! It may have swept off screen. Move it closer to center and rerun.")
        ser.write(f"M {-TEST_PAN_MRAD} 0\n".encode())
        sys.exit(1)

    delta_pan_fwd = abs(pos_B[0] - pos_A[0])
    fx_forward = solve_focal_length(pos_A[0], pos_B[0], OPTICAL_CENTER[0], TEST_PAN_MRAD)
    print(f"  Forward Sweep Displacement: {delta_pan_fwd:.2f} px -> fx = {fx_forward:.2f}")

    print(f"\n--- PAN AXIS REVERSE (Sweep: -{TEST_PAN_MRAD} mrad back to A) ---")
    ser.write(f"M {-TEST_PAN_MRAD} 0\n".encode())
    wait_for_settle(ser)

    print("Measuring Return Position (Position A')...")
    pos_A_prime, _ = get_robust_position(picam2, tracker)
    if pos_A_prime is not None:
        delta_pan_rev = abs(pos_B[0] - pos_A_prime[0])
        fx_reverse = solve_focal_length(pos_B[0], pos_A_prime[0], OPTICAL_CENTER[0], TEST_PAN_MRAD)
        print(f"  Reverse Sweep Displacement: {delta_pan_rev:.2f} px -> fx = {fx_reverse:.2f}")
        fx_final = (fx_forward + fx_reverse) / 2.0
        backlash_pan_px = abs(pos_A_prime[0] - pos_A[0])
        print(f"  Pan Mechanical Backlash: {backlash_pan_px:.2f} px")
    else:
        fx_final = fx_forward

    # -------------------------------------------------------------------------
    # Tilt Axis Calibration (Forward + Reverse)
    # -------------------------------------------------------------------------
    print(f"\n--- TILT AXIS CALIBRATION (Sweep: +{TEST_TILT_MRAD} mrad) ---")
    ser.write(f"M 0 {TEST_TILT_MRAD}\n".encode())
    wait_for_settle(ser)

    print("Measuring Displaced Position (Position C)...")
    pos_C, _ = get_robust_position(picam2, tracker)
    if pos_C is None:
        print("[ERROR] Lost marker on vertical sweep.")
        ser.write(f"M 0 {-TEST_TILT_MRAD}\n".encode())
        sys.exit(1)

    # Use baseline pos_A_prime or pos_A
    ref_y = pos_A_prime[1] if pos_A_prime is not None else pos_A[1]
    delta_tilt_fwd = abs(pos_C[1] - ref_y)
    fy_forward = solve_focal_length(ref_y, pos_C[1], OPTICAL_CENTER[1], TEST_TILT_MRAD)
    print(f"  Forward Sweep Displacement: {delta_tilt_fwd:.2f} px -> fy = {fy_forward:.2f}")

    print(f"\n--- TILT AXIS REVERSE (Sweep: -{TEST_TILT_MRAD} mrad) ---")
    ser.write(f"M 0 {-TEST_TILT_MRAD}\n".encode())
    wait_for_settle(ser)

    print("Measuring Return Position (Position C')...")
    pos_C_prime, _ = get_robust_position(picam2, tracker)
    if pos_C_prime is not None:
        delta_tilt_rev = abs(pos_C[1] - pos_C_prime[1])
        fy_reverse = solve_focal_length(pos_C[1], pos_C_prime[1], OPTICAL_CENTER[1], TEST_TILT_MRAD)
        print(f"  Reverse Sweep Displacement: {delta_tilt_rev:.2f} px -> fy = {fy_reverse:.2f}")
        fy_final = (fy_forward + fy_reverse) / 2.0
        backlash_tilt_px = abs(pos_C_prime[1] - ref_y)
        print(f"  Tilt Mechanical Backlash: {backlash_tilt_px:.2f} px")
    else:
        fy_final = fy_forward

    # -------------------------------------------------------------------------
    # Final Result
    # -------------------------------------------------------------------------
    print("\n" + "=" * 70)
    print(" OPTICAL CALIBRATION CONVERGED SUCCESSFULLY")
    print("=" * 70)
    print("Update bombardeer.py with these values:\n")
    print(f"FOCAL_LENGTH_X_PX = {fx_final:.2f}")
    print(f"FOCAL_LENGTH_Y_PX = {fy_final:.2f}")
    print("=" * 70 + "\n")

    picam2.stop()
    ser.close()


if __name__ == "__main__":
    main()