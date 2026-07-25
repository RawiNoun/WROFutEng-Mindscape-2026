#!/usr/bin/env python3
"""
pid_corner_drive.py -- standalone version of open_challenge.py's
latest logic. The kick loop (PID-driven normal driving) durations
were updated (0.4/0.2/0.4). The corner maneuver is now three phases:
reverse with opposite steering, then turn back with the original
steering, then drive straight until the wall is back in range.

No camera, no direction lock, no lap/round-timer bookkeeping. Runs
indefinitely until Ctrl+C.

Kick loop (continuous, IS normal driving)
------------------------------------------
kick_sign is (re)computed only at the START of each new KICK_STEER
phase, from the sign of the PID's live output at that instant. This
loop runs unconditionally and forever until a corner interrupts it:

  1. KICK_STEER         (0.4s) -- kick_sign * KICK_STEER_ANGLE
  2. KICK_COUNTER_STEER (0.2s) -- -kick_sign * KICK_COUNTER_STEER_ANGLE
  3. KICK_STEER2        (0.4s) -- kick_sign * KICK_STEER2_ANGLE
  -> loops back to step 1, recomputing kick_sign fresh each time.

Corner sequence (checked every loop, can interrupt any KICK_* phase)
-------------------------------------------------------------------
  1. CORNER_REVERSE (1.0s, FIXED) -- -TURN_SIGN * CORNER_REVERSE_ANGLE
                     (full lock, OPPOSITE the turn direction), driving
                     BACKWARD. Records triggered_side ("left"/"right")
                     -- whichever side actually tripped CORNER_TRIGGER_CM.
  2. CORNER_TURN    (0.8s, FIXED) -- +TURN_SIGN * CORNER_TURN_ANGLE
                     (full lock, OPPOSITE the reverse phase), driving
                     FORWARD.
  3. CORNER_STRAIGHT (open-loop, no fixed timer) -- steering centred,
                     driving FORWARD, until the triggered_side's
                     UNFILTERED reading drops under
                     CORNER_STRAIGHT_EXIT_CM (80cm).
  4. worker.request_reset(), corner counted, LED off, back into the
     KICK loop.

No camera / direction lock here
--------------------------------
open_challenge.py derives TURN_SIGN automatically from the camera at
the start of a round. There's no camera in this script, so TURN_SIGN
below is a manual constant -- set it to +1 (steer right into corners)
or -1 (steer left) to match whatever you're testing.

No cooldown after a corner -- corner detection goes live again the
instant the kick loop resumes.

Usage:
    python3 pid_corner_drive.py
Ctrl+C to stop.
"""

import time

from drive import Drive
from sensor_worker import SensorWorker
from gpiozero import LED

# ---------------- tunables ----------------
TURN_SIGN = +1                   # +1 = corner turn direction is right,
                                  # -1 = left (no camera here -- set manually)

BASE_SPEED = 0.3                # cruise speed used throughout the kick loop
TURN_SPEED = 0.5

CORNER_LED_PIN     = 22
CORNER_TRIGGER_CM  = 100         # either UNFILTERED reading above this => corner

# --- corner sequence: REVERSE (opposite steer, backward) -> TURN
# (original steer, forward) -> STRAIGHT (centred, until wall in range) ---
CORNER_REVERSE_ANGLE    = 100          # full lock, OPPOSITE TURN_SIGN
CORNER_REVERSE_DURATION = 0.8          # given (1 full second)
CORNER_REVERSE_SPEED    = TURN_SPEED   # ASSUMPTION -- not specified

CORNER_TURN_ANGLE       = 100          # full lock, OPPOSITE the reverse
                                        # phase (i.e. back to +TURN_SIGN)
CORNER_TURN_DURATION    = 1.4          # given
CORNER_TURN_SPEED       = TURN_SPEED   # ASSUMPTION -- not specified

CORNER_STRAIGHT_EXIT_CM = 80           # given
CORNER_STRAIGHT_SPEED   = BASE_SPEED   # ASSUMPTION -- not specified
CORNER_STRAIGHT_MAX_DURATION = 3.0     # safety fallback only, not requested

# --- kick loop (this IS normal driving, runs continuously) ---
KICK_STEER_ANGLE            = 100
KICK_STEER_DURATION         = 0.3
KICK_COUNTER_STEER_ANGLE    = KICK_STEER_ANGLE / 2
KICK_COUNTER_STEER_DURATION = 0.2
KICK_STEER2_ANGLE           = KICK_STEER_ANGLE
KICK_STEER2_DURATION        = 0.3

SNAPSHOT_TIMEOUT = 1.0           # seconds to wait for the sensor thread's
                                  # first reading before giving up at startup


def main():
    drive = Drive()
    corner_led = LED(CORNER_LED_PIN)
    corner_led.off()

    worker = SensorWorker()
    worker.start()

    print("[pid_corner_drive] waiting for first sensor reading...")
    snap = worker.get_latest(timeout=SNAPSHOT_TIMEOUT)
    if snap is None:
        print("[pid_corner_drive] no sensor reading arrived -- check ultrasonic wiring")
        worker.stop()
        return

    kick_sign = 1 if snap.steering > 0 else -1
    state = "KICK_STEER"
    phase_start = time.time()
    corners_done = 0
    triggered_side = None

    print("[pid_corner_drive] running. Ctrl+C to stop.")

    try:
        drive.forward(BASE_SPEED)

        while True:
            snap = worker.get_latest()
            seen_corner = (
                snap.fresh_left > CORNER_TRIGGER_CM
                or snap.fresh_right > CORNER_TRIGGER_CM
            )

            phase_elapsed = time.time() - phase_start

            if seen_corner and state in ("KICK_STEER", "KICK_COUNTER_STEER", "KICK_STEER2"):
                triggered_side = "left" if snap.fresh_left > CORNER_TRIGGER_CM else "right"
                state = "CORNER_REVERSE"
                phase_start = time.time()
                phase_elapsed = 0.0
                corner_led.on()
                drive.backward(CORNER_REVERSE_SPEED)
                print(f"[pid_corner_drive] corner detected on {triggered_side} "
                      f"-> CORNER_REVERSE (interrupting kick loop)")

            if state == "KICK_STEER":
                drive.set_steering(kick_sign * KICK_STEER_ANGLE)
                drive.set_speed(BASE_SPEED)
                if phase_elapsed > KICK_STEER_DURATION:
                    state = "KICK_COUNTER_STEER"
                    phase_start = time.time()

            elif state == "KICK_COUNTER_STEER":
                drive.set_steering(-kick_sign * KICK_COUNTER_STEER_ANGLE)
                drive.set_speed(BASE_SPEED)
                if phase_elapsed > KICK_COUNTER_STEER_DURATION:
                    state = "KICK_STEER2"
                    phase_start = time.time()

            elif state == "KICK_STEER2":
                drive.set_steering(kick_sign * KICK_STEER2_ANGLE)
                drive.set_speed(BASE_SPEED)
                if phase_elapsed > KICK_STEER2_DURATION:
                    kick_sign = 1 if snap.steering > 0 else -1
                    state = "KICK_STEER"
                    phase_start = time.time()

            elif state == "CORNER_REVERSE":
                drive.set_steering(-TURN_SIGN * CORNER_REVERSE_ANGLE)
                drive.set_speed(CORNER_REVERSE_SPEED)
                if phase_elapsed > CORNER_REVERSE_DURATION:
                    state = "CORNER_TURN"
                    phase_start = time.time()
                    drive.forward(CORNER_TURN_SPEED)  # flips direction
                                                       # pins back to forward
                    print("[pid_corner_drive] CORNER_REVERSE -> CORNER_TURN")

            elif state == "CORNER_TURN":
                drive.set_steering(TURN_SIGN * CORNER_TURN_ANGLE)
                drive.set_speed(CORNER_TURN_SPEED)
                if phase_elapsed > CORNER_TURN_DURATION:
                    state = "CORNER_STRAIGHT"
                    phase_start = time.time()
                    drive.set_steering(0)
                    drive.set_speed(CORNER_STRAIGHT_SPEED)
                    print("[pid_corner_drive] CORNER_TURN -> CORNER_STRAIGHT")

            elif state == "CORNER_STRAIGHT":
                drive.set_steering(0)
                drive.set_speed(CORNER_STRAIGHT_SPEED)

                watched = snap.fresh_left if triggered_side == "left" else snap.fresh_right
                wall_back_in_range = watched < CORNER_STRAIGHT_EXIT_CM
                timed_out = phase_elapsed > CORNER_STRAIGHT_MAX_DURATION

                if wall_back_in_range or timed_out:
                    if timed_out and not wall_back_in_range:
                        print(f"[pid_corner_drive] CORNER_STRAIGHT safety timeout "
                              f"({CORNER_STRAIGHT_MAX_DURATION}s) -- wall never came "
                              f"back under {CORNER_STRAIGHT_EXIT_CM}cm on "
                              f"{triggered_side}, resuming PID anyway")
                    corner_led.off()
                    corners_done += 1
                    worker.request_reset()
                    kick_sign = 1 if snap.steering > 0 else -1
                    state = "KICK_STEER"
                    phase_start = time.time()
                    print(f"[pid_corner_drive] corner #{corners_done} done "
                          f"({triggered_side} back to {watched:.1f}cm), resuming kick loop")

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[pid_corner_drive] interrupted")

    finally:
        worker.stop()
        worker.join(timeout=1.0)
        corner_led.off()
        corner_led.close()
        drive.stop()
        drive.cleanup()


if __name__ == "__main__":
    main()
