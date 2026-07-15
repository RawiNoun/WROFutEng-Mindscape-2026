# fixed_steering_drive_test.py

import time
from drive import Drive


SPEED = 0.20
RUN_TIME = 2.0


def run_test(drive, steering_value, label):
    print(f"[TEST] {label}")
    angle = drive.set_steering(steering_value)
    print(f"[INFO] steering={steering_value}, servo_angle={angle}")

    drive.forward(SPEED)
    time.sleep(RUN_TIME)

    drive.stop()
    time.sleep(1)


def main():
    drive = Drive()

    print("[INFO] Fixed steering drive test.")
    print("[INFO] Put the robot on the track with space around it.")
    print("[INFO] Press Ctrl+C to stop if needed.")

    try:
        run_test(drive, -60, "LEFT TURN")
        run_test(drive, 0, "STRAIGHT")
        run_test(drive, 60, "RIGHT TURN")

    except KeyboardInterrupt:
        print("\n[INFO] Manual stop.")

    finally:
        drive.cleanup()# fixed_steering_drive_test.py

import time
from drive import Drive


SPEED = 0.20
RUN_TIME = 2.0


def run_test(drive, steering_value, label):
    print(f"[TEST] {label}")
    angle = drive.set_steering(steering_value)
    print(f"[INFO] steering={steering_value}, servo_angle={angle}")

    drive.forward(SPEED)
    time.sleep(RUN_TIME)

    drive.stop()
    time.sleep(1)


def main():
    drive = Drive()

    print("[INFO] Fixed steering drive test.")
    print("[INFO] Put the robot on the track with space around it.")
    print("[INFO] Press Ctrl+C to stop if needed.")

    try:
        run_test(drive, -60, "LEFT TURN")
        run_test(drive, 0, "STRAIGHT")
        run_test(drive, 60, "RIGHT TURN")

    except KeyboardInterrupt:
        print("\n[INFO] Manual stop.")

    finally:
        drive.cleanup()
        print("[INFO] Clean stop.")


if __name__ == "__main__":
    main()
    print("[INFO] Clean stop.")


if __name__ == "__main__":
    main()