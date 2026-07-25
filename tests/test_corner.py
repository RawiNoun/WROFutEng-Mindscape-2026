#!/usr/bin/env python3
"""
test_corner_stop_led.py -- bench test: when a corner is detected, turn
on the corner LED (GPIO 22) and stop the motor. LED turns off and the
motor resumes once the corner clears. No steering/PID is applied to
the servo here -- this is purely to check the
corner-detection -> LED + motor-stop wiring in isolation before
trusting it inside the full open_challenge.py state machine.

Uses the same building blocks as open_challenge.py:
  - SensorWorker (sensor_worker.py) for ultrasonic + PID, running on
    its own thread, decoupled from this loop's timing
  - the UNFILTERED fresh_left / fresh_right from each snapshot for
    corner detection -- fast reaction, not lagged by the PID's
    anti-jump filter (see sensor_worker.py / ultrasonic_pid_steering.py
    for why filtered left_cm/right_cm would be slower to trigger)
  - the same CORNER_TRIGGER_CM / CORNER_EXIT_GRACE debounce pattern

RUN_DURATION_SECONDS = 4 by default, per your test-script convention.
NOTE: at BASE_SPEED, 4 seconds from a standing start on a straight
probably isn't enough to actually reach a corner -- bump this up (or
place the robot closer to a corner before running) if you want to
actually see it react. This is an auto-stop safety net, not a
"must finish within 4s" requirement.

Usage:
    python3 test_corner_stop_led.py
Ctrl+C to stop early.
"""

import time

from drive import Drive
from sensor_worker import SensorWorker
from gpiozero import LED

RUN_DURATION_SECONDS = 4        # auto-stop safety net -- see NOTE above

CORNER_LED_PIN    = 22
CORNER_TRIGGER_CM = 100         # either UNFILTERED reading above this => corner
CORNER_EXIT_GRACE = 0.4         # seconds both readings must stay clear before
                                 # we trust the corner is over and resume

BASE_SPEED = 0.30
SNAPSHOT_TIMEOUT = 1.0          # seconds to wait for the sensor thread's
                                 # first reading before giving up at startup


def main():
    drive = Drive()
    corner_led = LED(CORNER_LED_PIN)
    corner_led.off()

    worker = SensorWorker()
    worker.start()

    snap = worker.get_latest(timeout=SNAPSHOT_TIMEOUT)
    if snap is None:
        worker.stop()
        return


    corner_active = False
    corner_clear_since = None
    start_time = time.time()

    try:
        drive.forward(BASE_SPEED)

        while time.time() - start_time < RUN_DURATION_SECONDS:
            snap = worker.get_latest()

            seen_corner = (
                snap.fresh_left > CORNER_TRIGGER_CM
                or snap.fresh_right > CORNER_TRIGGER_CM
            )

            if seen_corner and not corner_active:
                corner_active = True
                corner_clear_since = None
                corner_led.on()
                drive.set_speed(0)

            elif corner_active and not seen_corner:
                if corner_clear_since is None:
                    corner_clear_since = time.time()
                elif time.time() - corner_clear_since > CORNER_EXIT_GRACE:
                    corner_active = False
                    corner_led.off()
                    drive.forward(BASE_SPEED)

            elif corner_active and seen_corner:
                corner_clear_since = None  # still in the corner, reset the debounce clock

            time.sleep(0.02)


    except KeyboardInterrupt:
        print("\n[test_corner_stop_led] interrupted")

    finally:
        worker.stop()
        worker.join(timeout=1.0)
        corner_led.off()
        corner_led.close()
        drive.stop()
        drive.cleanup()


if __name__ == "__main__":
    main()
