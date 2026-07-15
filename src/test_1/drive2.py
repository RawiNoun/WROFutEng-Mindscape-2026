import csv
import time
from datetime import datetime
from pathlib import Path

from gpiozero import Button

from camera import Camera
from drive import Drive
from vision_rois import analyze_rois


START_BUTTON_GPIO = 16

BASE_SPEED = 0.24
TURN_SPEED = 0.23

LOOP_DELAY = 0.03

KP = 2.4
KD = 0.9

MAX_STEERING = 70
DEFAULT_TURN_STEERING = 45
SHARP_TURN_STEERING = 70

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
    log_path = LOG_DIR / f"drive_asparagus_style_{timestamp}.csv"

    file = open(log_path, mode="w", newline="")
    writer = csv.writer(file)

    writer.writerow([
        "timestamp",
        "frame_id",
        "state",
        "left_white_ratio",
        "right_white_ratio",
        "left_wall_ratio",
        "right_wall_ratio",
        "left_wall_visible",
        "right_wall_visible",
        "left_wall_missing",
        "right_wall_missing",
        "only_left_wall_visible",
        "only_right_wall_visible",
        "walls_balanced",
        "wall_difference",
        "error",
        "derivative",
        "target_steering",
        "smoothed_steering",
        "servo_angle",
        "speed",
        "turn_count",
    ])

    return file, writer, log_path


def wait_for_start_button(button, drive):
    drive.stop()
    print("[INFO] Waiting for button on GPIO 16...")
    button.wait_for_press()
    time.sleep(0.3)
    print("[INFO] Started.")


def main():
    camera = Camera()
    drive = Drive()
    start_button = Button(START_BUTTON_GPIO, pull_up=False, bounce_time=0.08)

    state = "FOLLOW"

    frame_id = 0
    previous_error = 0
    smoothed_steering = 0

    turn_direction = 0
    turn_count = 0

    log_file, log_writer, log_path = create_log_file()

    print("[INFO] Starting camera...")
    camera.start()

    print("[INFO] Asparagus-style dynamic wall-visible logic active.")
    print("[INFO] left wall only  -> turn right")
    print("[INFO] right wall only -> turn left")
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

            camera.show_overlay(result["annotated"])

            error = result["error"]
            derivative = error - previous_error

            if state == "FOLLOW":
                previous_error = error

                if result["only_left_wall_visible"]:
                    state = "TURN_RIGHT"
                    turn_direction = 1

                elif result["only_right_wall_visible"]:
                    state = "TURN_LEFT"
                    turn_direction = -1

                if state == "FOLLOW":
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

                    speed = BASE_SPEED

                else:
                    target_steering = turn_direction * DEFAULT_TURN_STEERING
                    smoothed_steering = target_steering
                    speed = TURN_SPEED

            else:
                if result["wall_difference"] > 0.20:
                    target_steering = turn_direction * SHARP_TURN_STEERING
                else:
                    target_steering = turn_direction * DEFAULT_TURN_STEERING

                smoothed_steering = target_steering
                speed = TURN_SPEED

                if result["walls_balanced"]:
                    turn_count += 1
                    state = "FOLLOW"
                    previous_error = error
                    target_steering = 0
                    smoothed_steering = 0

            servo_angle = drive.set_steering(smoothed_steering)
            drive.forward(speed)

            timestamp = datetime.now().isoformat(timespec="milliseconds")

            log_writer.writerow([
                timestamp,
                frame_id,
                state,
                round(result["left_white_ratio"], 3),
                round(result["right_white_ratio"], 3),
                round(result["left_wall_ratio"], 3),
                round(result["right_wall_ratio"], 3),
                result["left_wall_visible"],
                result["right_wall_visible"],
                result["left_wall_missing"],
                result["right_wall_missing"],
                result["only_left_wall_visible"],
                result["only_right_wall_visible"],
                result["walls_balanced"],
                round(result["wall_difference"], 3),
                error,
                derivative,
                round(target_steering, 3),
                round(smoothed_steering, 3),
                round(servo_angle, 3),
                speed,
                turn_count,
            ])

            log_file.flush()

            if frame_id % PRINT_EVERY_N_FRAMES == 0:
                print(
                    f"frame={frame_id} | "
                    f"state={state} | "
                    f"turns={turn_count} | "
                    f"Lwall={result['left_wall_ratio']:.2f} | "
                    f"Rwall={result['right_wall_ratio']:.2f} | "
                    f"onlyL={result['only_left_wall_visible']} | "
                    f"onlyR={result['only_right_wall_visible']} | "
                    f"balanced={result['walls_balanced']} | "
                    f"err={error} | "
                    f"steer={smoothed_steering:.1f} | "
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
