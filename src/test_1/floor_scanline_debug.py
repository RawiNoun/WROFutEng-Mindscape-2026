# floor_scanline_debug.py

from datetime import datetime
from pathlib import Path
import time

import cv2
import numpy as np
from picamera2 import Picamera2, Preview

from camera_config import (
    WIDTH,
    HEIGHT,
    MAIN_FORMAT,
    LORES_FORMAT,
    USE_LARGEST_SENSOR_MODE,
    MAX_FOV,
    PREVIEW_TYPE,
)


OUTPUT_DIR = Path("debug_floor_scanline")
OUTPUT_DIR.mkdir(exist_ok=True)


def start_camera():
    picam2 = Picamera2()

    if USE_LARGEST_SENSOR_MODE:
        mode = picam2.sensor_modes[-1]
        main_size = mode["size"]
    else:
        main_size = (WIDTH, HEIGHT)

    config = picam2.create_preview_configuration(
        main={
            "size": main_size,
            "format": MAIN_FORMAT,
        },
        lores={
            "size": (WIDTH, HEIGHT),
            "format": LORES_FORMAT,
        },
    )

    picam2.configure(config)

    if PREVIEW_TYPE == "QTGL":
        picam2.start_preview(Preview.QTGL)
    elif PREVIEW_TYPE == "DRM":
        picam2.start_preview(Preview.DRM)

    picam2.start()
    time.sleep(2)

    if MAX_FOV:
        full_crop = picam2.camera_controls["ScalerCrop"][1]
        picam2.set_controls({"ScalerCrop": full_crop})

    return picam2


def get_frame(picam2):
    frame = picam2.capture_array("lores")
    frame = frame[:, :, :3]
    return frame


def build_floor_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Detect bright floor.
    # This is intentionally broad for the white WRO mat.
    lower_floor = np.array([0, 0, 80])
    upper_floor = np.array([180, 80, 255])

    floor_mask = cv2.inRange(hsv, lower_floor, upper_floor)

    kernel = np.ones((7, 7), np.uint8)

    floor_mask = cv2.morphologyEx(
        floor_mask,
        cv2.MORPH_CLOSE,
        kernel,
    )

    floor_mask = cv2.morphologyEx(
        floor_mask,
        cv2.MORPH_OPEN,
        kernel,
    )

    return floor_mask


def largest_white_segment(row):
    best_start = None
    best_end = None
    best_len = 0

    current_start = None

    for x, value in enumerate(row):
        if value > 0:
            if current_start is None:
                current_start = x
        else:
            if current_start is not None:
                current_end = x - 1
                current_len = current_end - current_start + 1

                if current_len > best_len:
                    best_len = current_len
                    best_start = current_start
                    best_end = current_end

                current_start = None

    if current_start is not None:
        current_end = len(row) - 1
        current_len = current_end - current_start + 1

        if current_len > best_len:
            best_len = current_len
            best_start = current_start
            best_end = current_end

    return best_start, best_end, best_len


def analyze_floor_scanlines(frame, floor_mask):
    height, width = floor_mask.shape
    annotated = frame.copy()

    scanline_ys = [
        int(height * 0.52),
        int(height * 0.60),
        int(height * 0.68),
        int(height * 0.76),
        int(height * 0.84),
    ]

    centers = []

    for y in scanline_ys:
        row = floor_mask[y, :]

        start_x, end_x, segment_len = largest_white_segment(row)

        cv2.line(
            annotated,
            (0, y),
            (width, y),
            (255, 255, 255),
            1,
        )

        if start_x is None or end_x is None:
            continue

        # Ignore tiny segments.
        if segment_len < 120:
            continue

        center_x = (start_x + end_x) // 2
        centers.append(center_x)

        cv2.circle(
            annotated,
            (start_x, y),
            6,
            (255, 255, 255),
            -1,
        )

        cv2.circle(
            annotated,
            (end_x, y),
            6,
            (255, 255, 255),
            -1,
        )

        cv2.circle(
            annotated,
            (center_x, y),
            5,
            (255, 255, 255),
            -1,
        )

    if centers:
        # Bottom scanlines matter more than far scanlines.
        target_x = int(np.median(centers[-3:]))
    else:
        target_x = width // 2

    target_y = int(height * 0.75)
    error = target_x - (width // 2)

    cv2.circle(
        annotated,
        (target_x, target_y),
        10,
        (255, 255, 255),
        -1,
    )

    cv2.line(
        annotated,
        (width // 2, height),
        (target_x, target_y),
        (255, 255, 255),
        2,
    )

    cv2.putText(
        annotated,
        f"target_x={target_x}, error={error}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )

    return annotated, target_x, error, centers


def save_debug_images(frame, floor_mask, annotated):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    original_path = OUTPUT_DIR / f"{timestamp}_original.png"
    mask_path = OUTPUT_DIR / f"{timestamp}_floor_mask.png"
    annotated_path = OUTPUT_DIR / f"{timestamp}_floor_scanline_annotated.png"

    cv2.imwrite(str(original_path), frame)
    cv2.imwrite(str(mask_path), floor_mask)
    cv2.imwrite(str(annotated_path), annotated)

    print(f"[INFO] Saved: {original_path}")
    print(f"[INFO] Saved: {mask_path}")
    print(f"[INFO] Saved: {annotated_path}")


def main():
    picam2 = start_camera()

    print("[INFO] Camera started.")
    print("[INFO] Capturing frame...")

    try:
        time.sleep(1)

        frame = get_frame(picam2)

        floor_mask = build_floor_mask(frame)

        annotated, target_x, error, centers = analyze_floor_scanlines(
            frame,
            floor_mask,
        )

        save_debug_images(
            frame,
            floor_mask,
            annotated,
        )

        print(f"[INFO] Floor centers: {centers}")
        print(f"[INFO] Target X: {target_x}")
        print(f"[INFO] Error from center: {error}")

    finally:
        picam2.stop_preview()
        picam2.stop()
        print("[INFO] Camera stopped cleanly.")


if __name__ == "__main__":
    main()