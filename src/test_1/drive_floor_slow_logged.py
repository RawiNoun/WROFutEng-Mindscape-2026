# drive_floor_slow_logged_v2.py

import csv
import time
from datetime import datetime
from pathlib import Path

import cv2

from camera import Camera
from drive import Drive
from vision_floor import analyze_floor


BASE_SPEED = 0.16
LOST_SPEED = 0.0
LOOP_DELAY = 0.08

STEERING_GAIN = 3.0
SMOOTHING_ALPHA = 0.45
MIN_STEERING = 12
MAX_STEERING = 70

SAVE_DEBUG_EVERY_N_FRAMES = 10

LOG_DIR = Path("logs")
DEBUG_DIR = Path("debug_drive")

LOG_DIR.mkdir(exist_ok=True)
DEBUG_DIR.mkdir(exist_ok=True)


def smooth_value(previous, current, alpha=SMOOTHING_ALPHA):
    return (1 - alpha) * previous + alpha * current


def apply_minimum_steering(value):
    if abs(value) < MIN_STEERING and abs(value) > 0:
        return MIN_STEERING if value > 0 else -MIN_STEERING

    return value


def limit_steering(value):
    return max(-MAX_STEERING, min(MAX_STEERING, value))


def create_log_file():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = LOG_DIR / f"drive_log_v2_{timestamp}.csv"

    file = open(log_path, mode="w", newline="")
    writer = csv.writer(file)

    writer.writerow([
        "timestamp",
        "frame_id",
        "target_x",
        "error",
        "raw_steering",
        "target_steering",
        "smoothed_steering",
        "servo_angle",
        "vision_ok",
        "centers",
        "speed",
    ])

    return file, writer, log_path


def save_debug_image(frame_id, annotated):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    image_path = DEBUG_DIR / f"drive_debug_v2_{timestamp}_frame_{frame_id}.png"
    cv2.imwrite(str(image_path), annotated)
    return image_path


def main():
    camera = Camera()
    drive = Drive()

    smoothed_steering = 0
    frame_id = 0

    log_file, log_writer, log_path = create_log_file()

    print("[INFO] Starting camera...")
    camera.start()

    print("[INFO] Starting slow floor-following drive v2.")
    print(f"[INFO] Log file: {log_path}")
    print("[INFO] Press Ctrl+C to stop.")

    try:
        while True:
            frame_start_time = time.time()

            frame = camera.get_frame()
            result = analyze_floor(frame)

            raw_steering = result["steering_value"]

            if result["vision_ok"]:
                target_steering = raw_steering * STEERING_GAIN
                target_steering = apply_minimum_steering(target_steering)
                target_steering = limit_steering(target_steering)

                smoothed_steering = smooth_value(
                    smoothed_steering,
                    target_steering,
                )

                smoothed_steering = limit_steering(smoothed_steering)

                servo_angle = drive.set_steering(smoothed_steering)
                speed = BASE_SPEED
                drive.forward(speed)

            else:
                target_steering = 0
                smoothed_steering = smooth_value(smoothed_steering, 0)

                servo_angle = drive.set_steering(0)
                speed = LOST_SPEED
                drive.forward(speed)

            timestamp = datetime.now().isoformat(timespec="milliseconds")

            log_writer.writerow([
                timestamp,
                frame_id,
                result["target_x"],
                result["error"],
                raw_steering,
                round(target_steering, 3),
                round(smoothed_steering, 3),
                round(servo_angle, 3),
                result["vision_ok"],
                result["centers"],
                speed,
            ])

            log_file.flush()

            if frame_id % SAVE_DEBUG_EVERY_N_FRAMES == 0:
                image_path = save_debug_image(frame_id, result["annotated"])
                print(f"[INFO] Saved debug image: {image_path}")

            print(
                f"frame={frame_id} | "
                f"error={result['error']} | "
                f"raw={raw_steering} | "
                f"target={target_steering:.1f} | "
                f"smooth={smoothed_steering:.1f} | "
                f"servo={servo_angle:.1f} | "
                f"speed={speed} | "
                f"ok={result['vision_ok']}"
            )

            frame_id += 1

            elapsed = time.time() - frame_start_time
            time.sleep(max(0, LOOP_DELAY - elapsed))

    except KeyboardInterrupt:
        print("\n[INFO] Manual stop.")

    finally:
        drive.cleanup()
        camera.stop()
        log_file.close()

        print("[INFO] Clean stop.")
        print(f"[INFO] Log saved at: {log_path}")
        print(f"[INFO] Debug images saved in: {DEBUG_DIR}")


if __name__ == "__main__":
    main()