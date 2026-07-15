# wall_mask_debug.py

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

from color_config import (
    LOWER_BLACK,
    UPPER_BLACK,
    MIN_WALL_AREA,
)


OUTPUT_DIR = Path("debug_wall")
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

    # Same working pipeline from your previous camera.py
    frame = frame[:, :, :3]

    return frame


def build_black_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    black_mask = cv2.inRange(
        hsv,
        LOWER_BLACK,
        UPPER_BLACK,
    )

    # Clean noise
    kernel = np.ones((5, 5), np.uint8)

    black_mask = cv2.morphologyEx(
        black_mask,
        cv2.MORPH_OPEN,
        kernel,
    )

    black_mask = cv2.morphologyEx(
        black_mask,
        cv2.MORPH_CLOSE,
        kernel,
    )

    return black_mask


def analyze_walls(frame, black_mask):
    height, width = black_mask.shape

    # Focus on lower part of the image.
    # This avoids reacting too much to far background and window shadows.
    crop_y_start = int(height * 0.35)

    cropped_mask = black_mask[crop_y_start:height, :]

    contours, _ = cv2.findContours(
        cropped_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    annotated = frame.copy()

    valid_contours = []

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < MIN_WALL_AREA:
            continue

        x, y, w, h = cv2.boundingRect(contour)

        # Shift y back because contour was found on cropped image
        y_original = y + crop_y_start

        valid_contours.append(
            {
                "area": area,
                "x": x,
                "y": y_original,
                "w": w,
                "h": h,
                "center_x": x + w // 2,
            }
        )

        cv2.rectangle(
            annotated,
            (x, y_original),
            (x + w, y_original + h),
            (255, 255, 255),
            2,
        )

    left_walls = [
        c for c in valid_contours
        if c["center_x"] < width // 2
    ]

    right_walls = [
        c for c in valid_contours
        if c["center_x"] >= width // 2
    ]

    left_x = None
    right_x = None

    if left_walls:
        # Use the right edge of the largest left wall
        largest_left = max(left_walls, key=lambda c: c["area"])
        left_x = largest_left["x"] + largest_left["w"]

    if right_walls:
        # Use the left edge of the largest right wall
        largest_right = max(right_walls, key=lambda c: c["area"])
        right_x = largest_right["x"]

    target_x = width // 2

    if left_x is not None and right_x is not None:
        target_x = (left_x + right_x) // 2
    elif left_x is not None:
        target_x = left_x + 180
    elif right_x is not None:
        target_x = right_x - 180

    target_y = int(height * 0.75)

    cv2.circle(
        annotated,
        (target_x, target_y),
        8,
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

    error = target_x - (width // 2)

    cv2.putText(
        annotated,
        f"target_x={target_x}, error={error}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )

    return annotated, valid_contours, target_x, error


def save_debug_images(frame, black_mask, annotated):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    original_path = OUTPUT_DIR / f"{timestamp}_original.png"
    mask_path = OUTPUT_DIR / f"{timestamp}_black_mask.png"
    annotated_path = OUTPUT_DIR / f"{timestamp}_annotated.png"

    cv2.imwrite(str(original_path), frame)
    cv2.imwrite(str(mask_path), black_mask)
    cv2.imwrite(str(annotated_path), annotated)

    print(f"[INFO] Saved: {original_path}")
    print(f"[INFO] Saved: {mask_path}")
    print(f"[INFO] Saved: {annotated_path}")


def main():
    picam2 = start_camera()

    print("[INFO] Camera started.")
    print("[INFO] Waiting 1 second before capture...")

    try:
        time.sleep(1)

        frame = get_frame(picam2)

        black_mask = build_black_mask(frame)

        annotated, contours, target_x, error = analyze_walls(
            frame,
            black_mask,
        )

        save_debug_images(
            frame,
            black_mask,
            annotated,
        )

        print(f"[INFO] Valid wall contours: {len(contours)}")
        print(f"[INFO] Target X: {target_x}")
        print(f"[INFO] Error from center: {error}")

    finally:
        picam2.stop_preview()
        picam2.stop()
        print("[INFO] Camera stopped cleanly.")


if __name__ == "__main__":
    main()