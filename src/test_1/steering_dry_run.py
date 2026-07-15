# steering_dry_run.py

import time

from camera import Camera
from drive import Drive
from vision_floor import analyze_floor


def main():
    camera = Camera()
    drive = Drive()

    print("[INFO] Starting camera...")
    camera.start()

    print("[INFO] Servo dry run started.")
    print("[INFO] Motor will NOT move.")
    print("[INFO] Press Ctrl+C to stop.")

    try:
        while True:
            frame = camera.get_frame()
            result = analyze_floor(frame)

            steering = result["steering_value"]

            if result["vision_ok"]:
                angle = drive.set_steering(steering)
            else:
                angle = drive.set_steering(0)

            print(
                f"target_x={result['target_x']} | "
                f"error={result['error']} | "
                f"steering={steering} | "
                f"servo_angle={angle} | "
                f"vision_ok={result['vision_ok']}"
            )

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping.")

    finally:
        drive.cleanup()
        camera.stop()
        print("[INFO] Clean stop.")


if __name__ == "__main__":
    main()