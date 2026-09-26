#!/usr/bin/env python3
"""
Bombardeer Automated Optical-to-Mechanical Calibration Suite
------------------------------------------------------------
- Automatically steps the Pan and Tilt axes through a calibration grid.
- Observes real pixel displacements via native Picamera2 ArUco tracking.
- Performs linear regression to compute STEPS_PER_PIXEL_PAN and STEPS_PER_PIXEL_TILT.
- Saves calibrated constants to calibration_data.json for instant ingestion.
"""

import os
import sys
import time
import json
import math
import numpy as np
import cv2
import serial
from picamera2 import Picamera2
from libcamera import Transform

CONFIG_OUTPUT_PATH = "/home/ben/Bombardeer/calibration_data.json"
DEFAULT_PORTS = ["/dev/ttyUSB0", "/dev/ttyACM0"]
BAUD_RATE = 115200

PAN_OFFSETS = [-1200, -600, 0, 600, 1200]
TILT_OFFSETS = [-1000, -500, 0, 500, 1000]

class TurretCalibrator:
    def __init__(self):
        self.ser = None
        self._connect_serial()
        
        # Init Camera
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (1280, 720), "format": "RGB888"},
            transform=Transform(hflip=True, vflip=True)
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1.0)
        
        # ArUco Detector
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters()
        self.params.errorCorrectionRate = 0.55
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def _connect_serial(self):
        for port in DEFAULT_PORTS:
            if os.path.exists(port):
                try:
                    self.ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
                    print(f"[Serial] Connected to ESP32 on {port}")
                    return
                except Exception:
                    continue
        print("[Error] Could not locate ESP32 serial port. Exiting.")
        sys.exit(1)

    def wait_for_motion_complete(self, timeout_sec=6.0):
        """Polls ESP32 with 'Q' until motors have completely finished deceleration."""
        start = time.time()
        time.sleep(0.1)
        while time.time() - start < timeout_sec:
            self.ser.write(b"Q\n")
            line = self.ser.readline().decode(errors="ignore").strip()
            if line.startswith("R "):
                parts = line.split()
                if len(parts) >= 2 and parts[1] == "0":
                    time.sleep(0.15)  # Allow physical inertia/vibration to settle
                    return True
            time.sleep(0.05)
        return False

    def move_absolute(self, pan, tilt):
        self.ser.write(f"A {pan} {tilt}\n".encode())
        self.wait_for_motion_complete()

    def get_stable_marker_pos(self, samples=5):
        """Samples multiple frames to filter optical noise and obtain subpixel centroid."""
        coords = []
        for _ in range(samples):
            req = self.picam2.capture_request()
            frame = req.make_array("main")
            req.release()

            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            enhanced = self.clahe.apply(gray)
            corners, ids, _ = self.detector.detectMarkers(enhanced)

            if ids is not None and len(ids) > 0:
                pts = corners[0].reshape((4, 2))
                cx = float(np.mean(pts[:, 0]))
                cy = float(np.mean(pts[:, 1]))
                coords.append((cx, cy))
            time.sleep(0.04)

        if len(coords) < 3:
            return None
        median_x = float(np.median([c[0] for c in coords]))
        median_y = float(np.median([c[1] for c in coords]))
        return (median_x, median_y)

    def run(self):
        print("\n=======================================================")
        print("    BOMBARDEER AUTOMATED CALIBRATION SYSTEM")
        print("=======================================================")
        print("1. Place an ArUco marker directly in view of the turret.")
        print("2. Ensure the turret has mechanical clearance to move.")
        input("\nPress ENTER when the marker is placed and visible...")

        # Baseline Verification
        print("\n[Step 1] Verifying target marker visibility...")
        pos = self.get_stable_marker_pos(samples=8)
        if pos is None:
            print("[ERROR] No ArUco marker detected! Check alignment and lighting.")
            return

        print(f" Marker detected at Optical Center: X={pos[0]:.1f}, Y={pos[1]:.1f}")
        
        # Zero origin
        self.ser.write(b"H\n")
        time.sleep(0.2)

        # ----------------------------------------------------
        # Pan Axis Calibration
        # ----------------------------------------------------
        print("\n[Step 2] Executing Pan Axis Test Sweeps...")
        pan_step_history = []
        pan_pixel_history = []

        for p_step in PAN_OFFSETS:
            print(f"  -> Moving Pan to {p_step} steps...")
            self.move_absolute(p_step, 0)
            target = self.get_stable_marker_pos()
            if target is None:
                print(f"  [Warning] Marker lost at pan step {p_step}. Skipping point.")
                continue
            pan_step_history.append(p_step)
            pan_pixel_history.append(target[0])
            print(f"     Recorded Pixel X: {target[0]:.2f}")

        # Return to origin
        self.move_absolute(0, 0)

        # ----------------------------------------------------
        # Tilt Axis Calibration
        # ----------------------------------------------------
        print("\n[Step 3] Executing Tilt Axis Test Sweeps...")
        tilt_step_history = []
        tilt_pixel_history = []

        for t_step in TILT_OFFSETS:
            print(f"  -> Moving Tilt to {t_step} steps...")
            self.move_absolute(0, t_step)
            target = self.get_stable_marker_pos()
            if target is None:
                print(f"  [Warning] Marker lost at tilt step {t_step}. Skipping point.")
                continue
            tilt_step_history.append(t_step)
            tilt_pixel_history.append(target[1])
            print(f"     Recorded Pixel Y: {target[1]:.2f}")

        # Return home
        self.move_absolute(0, 0)

        # ----------------------------------------------------
        # Regression & Math Processing
        # ----------------------------------------------------
        if len(pan_step_history) < 3 or len(tilt_step_history) < 3:
            print("[ERROR] Insufficient data points collected to perform calibration.")
            return

        # Linear regression: steps = slope * pixels + intercept
        # Since camera moves: positive turret pan shifts target left (negative pixel delta)
        poly_pan = np.polyfit(pan_pixel_history, pan_step_history, 1)
        poly_tilt = np.polyfit(tilt_pixel_history, tilt_step_history, 1)

        steps_per_pixel_pan = abs(float(poly_pan[0]))
        steps_per_pixel_tilt = abs(float(poly_tilt[0]))

        calib_data = {
            "timestamp": time.time(),
            "STEPS_PER_PIXEL_PAN": round(steps_per_pixel_pan, 3),
            "STEPS_PER_PIXEL_TILT": round(steps_per_pixel_tilt, 3),
            "PAN_INVERT": bool(poly_pan[0] > 0),
            "TILT_INVERT": bool(poly_tilt[0] > 0)
        }

        print("\n=======================================================")
        print("              CALIBRATION COMPLETE                     ")
        print("=======================================================")
        print(f" Pan Axis Ratio  : {calib_data['STEPS_PER_PIXEL_PAN']} steps / pixel")
        print(f" Tilt Axis Ratio : {calib_data['STEPS_PER_PIXEL_TILT']} steps / pixel")
        print("=======================================================")

        with open(CONFIG_OUTPUT_PATH, "w") as f:
            json.dump(calib_data, f, indent=4)
        print(f"[Success] Configuration saved to {CONFIG_OUTPUT_PATH}")

if __name__ == "__main__":
    calibrator = TurretCalibrator()
    calibrator.run()