#!/usr/bin/env python3
"""
Bombardeer One-Shot Target & Range Calibration Diagnostic
--------------------------------------------------------
Captures a single 1280x720 hardware frame, locates the ArUco marker,
measures its precise edge length in pixels, and calculates both ground-truth
focal length (if placed at a known distance) and estimated range.
"""

import sys
import argparse
import numpy as np
import cv2
import picamera2
from libcamera import Transform

# Defaults matching Bombardeer hardware specifications
DEFAULT_FOCAL_LENGTH_PX = 540.4   # Raspberry Pi Camera Module 3 Wide NoIR at 1280x720
DEFAULT_TARGET_WIDTH_MM = 100.0   # Physical width of printed ArUco marker edge
TARGET_MARKER_ID = 0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Single-shot tool to measure ArUco marker pixel dimensions and verify optical range estimation."
    )
    parser.add_argument(
        "--distance",
        type=float,
        default=None,
        help="Optional: Known physical distance in meters to calculate exact focal length (e.g. --distance 2.0)",
    )
    parser.add_argument(
        "--width",
        type=float,
        default=DEFAULT_TARGET_WIDTH_MM,
        help=f"Physical marker edge width in mm (default: {DEFAULT_TARGET_WIDTH_MM} mm)",
    )
    parser.add_argument(
        "--focal-length",
        type=float,
        default=DEFAULT_FOCAL_LENGTH_PX,
        help=f"Camera focal length in pixels for distance projection (default: {DEFAULT_FOCAL_LENGTH_PX} px)",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # 1. Initialize Camera Module 3 in Native 1280x720 Mode
    print("[1/3] Initializing Camera Module via Picamera2...")
    try:
        picam2 = picamera2.Picamera2()
        config = picam2.create_video_configuration(
            sensor={"output_size": (2304, 1296)},
            main={"size": (1280, 720), "format": "RGB888"},
            transform=Transform(hflip=True, vflip=True)
        )
        picam2.configure(config)
        picam2.start()
    except Exception as e:
        print(f"[ERROR] Failed to initialize camera: {e}")
        sys.exit(1)

    # 2. Acquire a Single Frame
    print("[2/3] Capturing diagnostic frame...")
    try:
        req = picam2.capture_request()
        frame = req.make_array("main")
        req.release()
    finally:
        picam2.stop()

    # 3. Detect ArUco Marker and Compute Geometry
    print("[3/3] Processing ArUco geometry (DICT_4X4_50)...")
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None or len(ids) == 0:
        print("\n" + "=" * 60)
        print("[FAIL] No ArUco marker detected!")
        print("Ensure the marker (DICT_4X4_50, ID: 0) is well-lit, unobstructed,")
        print("and held steady within the camera's field of view.")
        print("=" * 60 + "\n")
        sys.exit(1)

    # Filter for ID 0 or take primary target
    target_idx = 0
    if TARGET_MARKER_ID is not None:
        flat_ids = ids.flatten()
        if TARGET_MARKER_ID in flat_ids:
            target_idx = int(np.where(flat_ids == TARGET_MARKER_ID)[0][0])

    pts = corners[target_idx].reshape((4, 2))
    detected_id = int(ids.flatten()[target_idx])

    # Calculate both edge lengths to account for aspect tilt/perspective
    edge_top = np.linalg.norm(pts[0] - pts[1])
    edge_right = np.linalg.norm(pts[1] - pts[2])
    edge_bottom = np.linalg.norm(pts[2] - pts[3])
    edge_left = np.linalg.norm(pts[3] - pts[0])

    avg_px_w = float((edge_top + edge_right + edge_bottom + edge_left) / 4.0)

    # Range calculation via pinhole projection: D = (W_real * f_px) / (W_px * 1000)
    estimated_dist_m = (args.width * args.focal_length) / (avg_px_w * 1000.0)
    estimated_dist_ft = estimated_dist_m * 3.28084

    print("\n" + "=" * 60)
    print("           TARGET MEASUREMENT RESULTS")
    print("=" * 60)
    print(f" Target Marker ID       : {detected_id}")
    print(f" Measured Target Width  : {avg_px_w:.2f} pixels")
    print(f" Individual Edges       : T={edge_top:.1f}px, R={edge_right:.1f}px, B={edge_bottom:.1f}px, L={edge_left:.1f}px")
    print("-" * 60)
    print(f" Using Focal Length     : {args.focal_length:.1f} px")
    print(f" Target Physical Width  : {args.width:.1f} mm")
    print(f" Estimated Distance     : {estimated_dist_m:.3f} meters ({estimated_dist_ft:.2f} feet)")

    # Optional: Back-calculate exact focal length if a ground-truth distance was provided
    if args.distance is not None and args.distance > 0:
        known_distance_mm = args.distance * 1000.0
        solved_focal_length = (known_distance_mm * avg_px_w) / args.width
        mrad_per_pixel = 1000.0 / solved_focal_length
        print("-" * 60)
        print("          CALIBRATED INTRINSICS (SOLVED)")
        print("-" * 60)
        print(f" Ground Truth Distance  : {args.distance:.3f} meters ({args.distance * 3.28084:.2f} feet)")
        print(f" Solved FOCAL_LENGTH_PX : {solved_focal_length:.2f}")
        print(f" Solved MRAD_PER_PIXEL  : {mrad_per_pixel:.4f} mrad/px")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    main()