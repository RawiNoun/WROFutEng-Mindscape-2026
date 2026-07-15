# vision_floor_test.py

from datetime import datetime
from pathlib import Path
import time

import cv2
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

from vision_floor import analyze_floor


OUTPUT_DIR = Path("debug_vision_floor")
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


def save_outputs(result):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    annotated_path = OUTPUT_DIR / f"{timestamp}_annotated.png"
    mask_path = OUTPUT_DIR / f"{timestamp}_floor_mask.png"

    cv2.imwrite(str(annotated_path), result["annotated"])
    cv2.imwrite(str(mask_path), result["floor_mask"])

    print(f"[INFO] Saved: {annotated_path}")
    print(f"[INFO] Saved: {mask_path}")


def main():
    picam2 = start_camera()

    print("[INFO] Camera started.")
    print("[INFO] Capturing 5 frames...")

    try:
        for i in range(5):
            frame = get_frame(picam2)
            result = analyze_floor(frame)

            print(
                f"[FRAME {i}] "
                f"target_x={result['target_x']} | "
                f"error={result['error']} | "
                f"steering={result['steering_value']} | "
                f"vision_ok={result['vision_ok']} | "
                f"centers={result['centers']}"
            )

            save_outputs(result)

            time.sleep(0.5)

    finally:
        picam2.stop_preview()
        picam2.stop()
        print("[INFO] Camera stopped cleanly.")


if __name__ == "__main__":
    main()