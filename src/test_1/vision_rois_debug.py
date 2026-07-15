# vision_rois_debug.py

from datetime import datetime
from pathlib import Path
import time

import cv2

from camera import Camera
from vision_rois import analyze_rois


OUTPUT_DIR = Path("debug_rois")
OUTPUT_DIR.mkdir(exist_ok=True)


def save_outputs(result):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

    annotated_path = OUTPUT_DIR / f"{timestamp}_annotated.png"
    mask_path = OUTPUT_DIR / f"{timestamp}_white_mask.png"

    cv2.imwrite(str(annotated_path), result["annotated"])
    cv2.imwrite(str(mask_path), result["white_mask"])

    print(f"[INFO] Saved: {annotated_path}")
    print(f"[INFO] Saved: {mask_path}")


def main():
    camera = Camera()

    print("[INFO] Starting camera...")
    camera.start()

    try:
        for i in range(5):
            frame = camera.get_frame()
            result = analyze_rois(frame)

            print(
                f"[FRAME {i}] "
                f"L={result['left_ratio']:.2f} | "
                f"R={result['right_ratio']:.2f} | "
                f"F={result['front_ratio']:.2f} | "
                f"error={result['error']} | "
                f"front_blocked={result['front_blocked']} | "
                f"left_open={result['left_open']} | "
                f"right_open={result['right_open']}"
            )

            save_outputs(result)
            time.sleep(0.5)

    finally:
        camera.stop()
        print("[INFO] Camera stopped.")


if __name__ == "__main__":
    main()