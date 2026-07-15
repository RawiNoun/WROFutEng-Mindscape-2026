# drive_open_pd_rois_logged.py

import csv
import time
from datetime import datetime
from pathlib import Path

from gpiozero import Button

from camera import Camera
from drive import Drive
from vision_rois import analyze_rois


START_BUTTON_GPIO = 16

BASE_SPEED = 0.30
TURN_SPEED = 0.25
LOST_SPEED = 0.0

LOOP_DELAY = 0.03

KP = 2.4
KD = 0.9

MAX_STEERING = 70
STEERING_SMOOTHING_ALPHA = 0.45

PRINT_EVERY_N_FRAMES = 5

LOG_DIR = Path("logs")
LOG_DIR.mkdir(exist_ok=True)


def clamp(value, low, high):
    return max(low, min(high, value))


def smooth(previous, current, alpha):
    return (1 - alpha) * previous + alpha * current


def create_log_file():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = LOG_DIR / f"drive_pd_rois_logged_{timestamp}.csv"

    file = open(log_path, mode="w", newline="")
    writer = csv.writer(file)

    writer.writerow([
        "timestamp",
        "frame_id",
        "state",
        "left_ratio",
        "right_ratio",
        "front_ratio",
        "front_blocked",
        "left_open",
        "right_open",
        "error",
        "derivative",
        "target_steering",
        "smoothed_steering",
        "servo_angle",
        "speed",
    ])

    return file, writer, log_path


def wait_for_start_button(button, drive):
    drive.stop()

    print("[INFO] Waiting for button on GPIO 16...")
    print("[INFO] First press = start.")

    button.wait_for_press()
    time.sleep(0.3)

    print("[INFO] Started.")


def main():
    camera = Camera()
    drive = Drive()
    start_button = Button(START_BUTTON_GPIO, pull_up=False, bounce_time=0.08)

    frame_id = 0
    previous_error = 0
    smoothed_steering = 0

    log_file, log_writer, log_path = create_log_file()

    print("[INFO] Starting camera...")
    camera.start()

    print("[INFO] ROI PD driving with front/side turn logic.")
    print("[INFO] First button press starts. Second button press stops.")
    print(f"[INFO] Log file: {log_path}")

    try:
        wait_for_start_button(start_button, drive)

        while True:
            if start_button.is_pressed:
                print("[INFO] Stop button press detected.")
                break

            loop_start = time.time()

            frame = camera.get_frame()
            result = analyze_rois(frame)

            error = result["error"]
            derivative = error - previous_error
            previous_error = error

            target_steering = (KP * error) + (KD * derivative)
            target_steering = clamp(
                target_steering,
                -MAX_STEERING,
                MAX_STEERING,
            )

            smoothed_steering = smooth(
                smoothed_steering,
                target_steering,
                STEERING_SMOOTHING_ALPHA,
            )

            smoothed_steering = clamp(
                smoothed_steering,
                -MAX_STEERING,
                MAX_STEERING,
            )

            if result["front_blocked"]:
                if result["left_open"]:
                    state = "TURN_LEFT"
                    target_steering = -MAX_STEERING

                elif result["right_open"]:
                    state = "TURN_RIGHT"
                    target_steering = MAX_STEERING

                else:
                    state = "TURN_UNKNOWN"
                    target_steering = smoothed_steering

                smoothed_steering = target_steering
                servo_angle = drive.set_steering(smoothed_steering)
                speed = TURN_SPEED

            else:
                state = "FOLLOW"
                servo_angle = drive.set_steering(smoothed_steering)
                speed = BASE_SPEED

            drive.forward(speed)

            timestamp = datetime.now().isoformat(timespec="milliseconds")

            log_writer.writerow([
                timestamp,
                frame_id,
                state,
                round(result["left_ratio"], 3),
                round(result["right_ratio"], 3),
                round(result["front_ratio"], 3),
                result["front_blocked"],
                result["left_open"],
                result["right_open"],
                error,
                derivative,
                round(target_steering, 3),
                round(smoothed_steering, 3),
                round(servo_angle, 3),
                speed,
            ])
            log_file.flush()

            if frame_id % PRINT_EVERY_N_FRAMES == 0:
                print(
                    f"frame={frame_id} | "
                    f"state={state} | "
                    f"L={result['left_ratio']:.2f} | "
                    f"R={result['right_ratio']:.2f} | "
                    f"F={result['front_ratio']:.2f} | "
                    f"blocked={result['front_blocked']} | "
                    f"left_open={result['left_open']} | "
                    f"right_open={result['right_open']} | "
                    f"err={error} | "
                    f"d={derivative} | "
                    f"target={target_steering:.1f} | "
                    f"smooth={smoothed_steering:.1f} | "
                    f"servo={servo_angle:.1f} | "
                    f"speed={speed}"
                )

            frame_id += 1

            elapsed = time.time() - loop_start
            time.sleep(max(0, LOOP_DELAY - elapsed))

    except KeyboardInterrupt:
        print("\n[INFO] Manual stop.")

    finally:
        drive.cleanup()
        camera.stop()
        start_button.close()
        log_file.close()

        print("[INFO] Clean stop.")
        print(f"[INFO] Log saved at: {log_path}")


if __name__ == "__main__":
    main()
