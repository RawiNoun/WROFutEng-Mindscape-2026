# camera_reference_test.py

from datetime import datetime
from pathlib import Path
import sys
import select
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


SCREENSHOT_DIR = Path("screenshots")
SCREENSHOT_DIR.mkdir(exist_ok=True)


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


def save_screenshot(frame):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = SCREENSHOT_DIR / f"camera_reference_{timestamp}.png"

    # Your lores frame comes from XRGB8888 and is reduced to 3 channels.
    # Save directly because this was your previous working color pipeline.
    cv2.imwrite(str(filename), frame)

    print(f"[INFO] Screenshot saved: {filename}")


def terminal_input_available():
    return select.select([sys.stdin], [], [], 0)[0]


def main():
    picam2 = start_camera()

    printed_info = False

    print("[INFO] Camera started.")
    print("[INFO] Picamera2 preview window should be visible.")
    print("[INFO] Type 's' then Enter to save screenshot.")
    print("[INFO] Type 'q' then Enter to quit.")

    try:
        while True:
            frame = picam2.capture_array("lores")

            # Same working pipeline from your old camera.py
            frame = frame[:, :, :3]

            if not printed_info:
                print(f"[INFO] Frame shape: {frame.shape}")
                print(f"[INFO] WIDTH={WIDTH}, HEIGHT={HEIGHT}")
                print(f"[INFO] MAIN_FORMAT={MAIN_FORMAT}")
                print(f"[INFO] LORES_FORMAT={LORES_FORMAT}")
                printed_info = True

            if terminal_input_available():
                command = sys.stdin.readline().strip().lower()

                if command == "s":
                    save_screenshot(frame)

                elif command == "q":
                    print("[INFO] Quitting.")
                    break

            time.sleep(0.03)

    finally:
        picam2.stop_preview()
        picam2.stop()
        print("[INFO] Camera stopped cleanly.")


if __name__ == "__main__":
    main()