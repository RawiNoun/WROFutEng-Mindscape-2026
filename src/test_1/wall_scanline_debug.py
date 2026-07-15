# wall_scanline_debug.py

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
)


OUTPUT_DIR = Path("debug_scanline")
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


def build_black_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    black_mask = cv2.inRange(
        hsv,
        LOWER_BLACK,
        UPPER_BLACK,
    )

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


def find_wall_edges_on_row(mask, y, black_threshold=180):
    height, width = mask.shape
    row = mask[y, :]

    left_edge = None
    right_edge = None

    # Search from left until black wall ends
    in_left_black = False

    for x in range(0, width // 2):
        if row[x] > black_threshold:
            in_left_black = True

        elif in_left_black and row[x] <= black_threshold:
            left_edge = x
            break

    # Search from right until black wall ends
    in_right_black = False

    for x in range(width - 1, width // 2, -1):
        if row[x] > black_threshold:
            in_right_black = True

        elif in_right_black and row[x] <= black_threshold:
            right_edge = x
            break

    return left_edge, right_edge


def analyze_scanlines(frame, black_mask):
    height, width = black_mask.shape
    annotated = frame.copy()

    # Use lower-middle rows only.
    # Avoid very top: background/window.
    # Avoid very bottom: robot shadow / floor distortion.
    scanline_ys = [
        int(height * 0.48),
        int(height * 0.56),
        int(height * 0.64),
        int(height * 0.72),
    ]

    centers = []

    for y in scanline_ys:
        left_edge, right_edge = find_wall_edges_on_row(
            black_mask,
            y,
        )

        cv2.line(
            annotated,
            (0, y),
            (width, y),
            (255, 255, 255),
            1,
        )

        if left_edge is not None:
            cv2.circle(
                annotated,
                (left_edge, y),
                6,
                (255, 255, 255),
                -1,
            )

        if right_edge is not None:
            cv2.circle(
                annotated,
                (right_edge, y),
                6,
                (255, 255, 255),
                -1,
            )

        if left_edge is not None and right_edge is not None:
            center_x = (left_edge + right_edge) // 2
            centers.append(center_x)

            cv2.circle(
                annotated,
                (center_x, y),
                5,
                (255, 255, 255),
                -1,
            )

    if centers:
        target_x = int(np.median(centers))
    else:
        target_x = width // 2

    target_y = int(height * 0.68)
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


def save_debug_images(frame, black_mask, annotated):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    original_path = OUTPUT_DIR / f"{timestamp}_original.png"
    mask_path = OUTPUT_DIR / f"{timestamp}_black_mask.png"
    annotated_path = OUTPUT_DIR / f"{timestamp}_scanline_annotated.png"

    cv2.imwrite(str(original_path), frame)
    cv2.imwrite(str(mask_path), black_mask)
    cv2.imwrite(str(annotated_path), annotated)

    print(f"[INFO] Saved: {original_path}")
    print(f"[INFO] Saved: {mask_path}")
    print(f"[INFO] Saved: {annotated_path}")


def main():
    picam2 = start_camera()

    print("[INFO] Camera started.")
    print("[INFO] Waiting before capture...")

    try:
        time.sleep(1)

        frame = get_frame(picam2)
        black_mask = build_black_mask(frame)

        annotated, target_x, error, centers = analyze_scanlines(
            frame,
            black_mask,
        )

        save_debug_images(
            frame,
            black_mask,
            annotated,
        )

        print(f"[INFO] Scanline centers: {centers}")
        print(f"[INFO] Target X: {target_x}")
        print(f"[INFO] Error from center: {error}")

    finally:
        picam2.stop_preview()
        picam2.stop()
        print("[INFO] Camera stopped cleanly.")


if __name__ == "__main__":
    main()