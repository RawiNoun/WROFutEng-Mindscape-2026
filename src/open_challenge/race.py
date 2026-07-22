#!/usr/bin/env python3
"""
race.py  --  WRO Open Challenge, full run. Uses YOUR Camera + Drive classes.

Corner method (state machine):
  STRAIGHT (wall centering) -> line seen -> APPROACHING (drive TURN_DELAY_S)
  -> TURNING_90 (full lock for TURN_90_DURATION_S at TURN_SPEED)
  -> STRAIGHT again. First line color sets direction for the
  whole run. 12 corners = 3 laps -> FINISHING -> stop (+ LED).

Modes:
    python3 race.py            # race
    python3 race.py preview    # vision only, no motors
    python3 race.py snapshot   # save one annotated frame + print readings

== TUNE THESE (in order) ==
  1. TURN_DELAY_S        where the turn starts after the line is seen
  2. TURN_90_DURATION_S  how much it rotates (exit parallel to new straight)
  3. TURN_SPEED          only if the turn radius is wrong at full lock
Re-check timings after charging the battery (fresh battery turns faster).
"""

import sys
import time
from time import monotonic

import cv2
import numpy as np
from gpiozero import LED, Button

from camera import Camera
from drive import Drive


# ════════════════ CORNER TUNABLES ════════════════
TURN_DELAY_S        = 0      # drive straight this long after seeing the line (0 = turn immediately)
TURN_90_DURATION_S  = 1    # full-lock turn time  <-- MAIN corner knob
TURN_SPEED          = 0.35   # motor speed during the turn (0..1)
STOP_BEFORE_TURN_S  = 0.10   # (unused now: no stop before the turn)
CORNER_COOLDOWN_S   = 0.6    # ignore lines this long after a turn (was 1.0)
FIRST_ORANGE_MEANS  = "RIGHT"  # orange first -> clockwise -> right turns

# ════════════════ DRIVING ════════════════
STRAIGHT_SPEED      = 0.25
REVERSE_SPEED       = 0.30
RECOVER_SPEED       = 0.35

STEER_CENTER = 0
STEER_LEFT   = -65            # flip signs if your car steers mirrored
STEER_RIGHT  = 50

# wall centering (camera)
WALL_ROI_TOP    = 0.15
WALL_ROI_BOTTOM = 0.40
WALL_ROI_WIDTH  = 0.20
WALL_L_MAX      = 110         # LAB L below this = black wall (your calibration)
CAM_STEER_DEADBAND = 0.06
CAM_STEER_GAIN     = 90.0
CAM_MAX_OFFSET     = 70
STEER_SMOOTH_ALPHA = 0.35

# front-wall box: if a wall fills this, BACK UP then steer (user position)
FRONT_ROI_TOP    = 0.30
FRONT_ROI_BOTTOM = 0.50
FRONT_ROI_LEFT   = 0.20
FRONT_ROI_RIGHT  = 0.80
FRONT_WALL_THRESHOLD = 0.30   # fraction of the box that must be wall (lower = detect earlier)
BACKUP_ONE_SIDE_S = 0.4       # backup time when ONE half sees the wall (was 0.5)
BACKUP_BOTH_S     = 0.7       # backup time when BOTH halves see the wall (was 1)
LINE_BLIND_AFTER_REVERSE_S = 1   # ignore the line box this long after any backup
LINE_BLIND_AFTER_CORNER_S  = 2.0 # ignore the line box this long after a corner is detected

# pre-corner box: checked ONLY when a corner is detected, BEFORE turning.
# y 80-120 px; x is 10% narrower than the front box on EACH side (0.20 - 0.80).
PRECORNER_Y_TOP_PX    = 80
PRECORNER_Y_BOTTOM_PX = 120
PRECORNER_X_LEFT     = FRONT_ROI_LEFT + 0.15    # 0.40
PRECORNER_X_RIGHT    = FRONT_ROI_RIGHT - 0.15   # 0.60
PRECORNER_WALL_THRESHOLD = 0.30
PRECORNER_REVERSE_S  = 0.2     # (unused for corners now: they reverse until clear)
BACKUP_MAX_S         = 0.4     # safety cap: never reverse longer than this
CORNER_WAIT_S        = 1.0     # (unused now: no stop before the turn)

# stuck detection + recovery
STUCK_FRAME_DIFF_THRESHOLD = 3.0
STUCK_TIME_S               = 1.5
REVERSE_DURATION_S         = 1.5
RECOVER_FORWARD_DURATION_S = 1

# line detection
LINE_ROI_TOP    = 0.65
LINE_ROI_BOTTOM = 0.90
LINE_ROI_LEFT   = 0.10
LINE_ROI_RIGHT  = 0.90
ORANGE_LO = np.array([  5, 100,  80]);  ORANGE_HI = np.array([ 25, 255, 255])
BLUE_LO   = np.array([ 90,  80,  40]);  BLUE_HI   = np.array([130, 255, 255])
LINE_MIN_PIXELS = 150

TOTAL_CORNERS = 12
STOP_DELAY_S  = 0.5
LOOP_DT       = 0.06

LED_PIN = 22                  # finish LED (pin map: motor=17/27/25, servo=24, button=16)
START_BUTTON_GPIO = 16        # press to start the run; press again to stop
# ══════════════════════════════════════════════════


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wall_rois(frame):
    h, w = frame.shape[:2]
    t, b = int(WALL_ROI_TOP * h), int(WALL_ROI_BOTTOM * h)
    ww = int(WALL_ROI_WIDTH * w)
    return frame[t:b, 0:ww], frame[t:b, w - ww:w]


def wall_amount(roi_bgr):
    lab = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2LAB)
    mask = lab[:, :, 0] < WALL_L_MAX
    return float(np.count_nonzero(mask)) / mask.size


def front_wall_amount(frame):
    """Fraction (0..1) of the front box that is black wall."""
    h, w = frame.shape[:2]
    t, b = int(FRONT_ROI_TOP * h), int(FRONT_ROI_BOTTOM * h)
    l, r = int(FRONT_ROI_LEFT * w), int(FRONT_ROI_RIGHT * w)
    lab = cv2.cvtColor(frame[t:b, l:r], cv2.COLOR_BGR2LAB)
    mask = lab[:, :, 0] < WALL_L_MAX
    return float(np.count_nonzero(mask)) / mask.size


def front_wall_halves(frame):
    """(left_fraction, right_fraction) of the front box, split in half."""
    h, w = frame.shape[:2]
    t, b = int(FRONT_ROI_TOP * h), int(FRONT_ROI_BOTTOM * h)
    l, r = int(FRONT_ROI_LEFT * w), int(FRONT_ROI_RIGHT * w)
    mid = (l + r) // 2
    lab_l = cv2.cvtColor(frame[t:b, l:mid], cv2.COLOR_BGR2LAB)
    lab_r = cv2.cvtColor(frame[t:b, mid:r], cv2.COLOR_BGR2LAB)
    ml = lab_l[:, :, 0] < WALL_L_MAX
    mr = lab_r[:, :, 0] < WALL_L_MAX
    return (float(np.count_nonzero(ml)) / ml.size,
            float(np.count_nonzero(mr)) / mr.size)


def precorner_wall_amount(frame):
    """Fraction (0..1) of the pre-corner strip (x 0.15-0.85, y 0-150px) that is wall."""
    h, w = frame.shape[:2]
    y1 = max(0, PRECORNER_Y_TOP_PX)
    y2 = min(h, PRECORNER_Y_BOTTOM_PX)
    x1 = int(PRECORNER_X_LEFT * w)
    x2 = int(PRECORNER_X_RIGHT * w)
    lab = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2LAB)
    mask = lab[:, :, 0] < WALL_L_MAX
    return float(np.count_nonzero(mask)) / mask.size


def line_color(frame):
    h, w = frame.shape[:2]
    t, b = int(LINE_ROI_TOP * h), int(LINE_ROI_BOTTOM * h)
    l, r = int(LINE_ROI_LEFT * w), int(LINE_ROI_RIGHT * w)
    hsv = cv2.cvtColor(frame[t:b, l:r], cv2.COLOR_BGR2HSV)
    orange = cv2.countNonZero(cv2.inRange(hsv, ORANGE_LO, ORANGE_HI))
    blue   = cv2.countNonZero(cv2.inRange(hsv, BLUE_LO, BLUE_HI))
    if orange >= LINE_MIN_PIXELS and orange >= blue:
        return "ORANGE"
    if blue >= LINE_MIN_PIXELS:
        return "BLUE"
    return None


def frame_diff(a, b):
    ga = cv2.resize(cv2.cvtColor(a, cv2.COLOR_BGR2GRAY), (80, 60))
    gb = cv2.resize(cv2.cvtColor(b, cv2.COLOR_BGR2GRAY), (80, 60))
    return cv2.absdiff(ga, gb).mean()


def annotate(frame, extra=""):
    out = frame.copy()
    h, w = out.shape[:2]
    t, b = int(WALL_ROI_TOP * h), int(WALL_ROI_BOTTOM * h)
    ww = int(WALL_ROI_WIDTH * w)
    lroi, rroi = wall_rois(frame)
    Lw, Rw = wall_amount(lroi), wall_amount(rroi)
    cv2.rectangle(out, (0, t), (ww, b), (0, 255, 0), 2)
    cv2.rectangle(out, (w - ww, t), (w, b), (0, 255, 0), 2)
    cv2.putText(out, f"{Lw:.2f}", (4, t + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(out, f"{Rw:.2f}", (w - ww + 4, t + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    # front-wall box (yellow), split into halves with per-half fractions
    ft, fb = int(FRONT_ROI_TOP * h), int(FRONT_ROI_BOTTOM * h)
    fl, fr = int(FRONT_ROI_LEFT * w), int(FRONT_ROI_RIGHT * w)
    fmid = (fl + fr) // 2
    FwL, FwR = front_wall_halves(frame)
    cv2.rectangle(out, (fl, ft), (fr, fb), (0, 255, 255), 2)
    cv2.line(out, (fmid, ft), (fmid, fb), (0, 255, 255), 1)
    # inner cut lines (magenta): the pre-corner x boundaries drawn inside the
    # front box, so you can see the half-split AND the inner corner region.
    # Position controlled by PRECORNER_X_LEFT / PRECORNER_X_RIGHT
    # (= FRONT_ROI_LEFT + 0.15 / FRONT_ROI_RIGHT - 0.15 -- change the 0.15 offsets).
    icl = int(PRECORNER_X_LEFT * w)
    icr = int(PRECORNER_X_RIGHT * w)
    cv2.line(out, (icl, ft), (icl, fb), (255, 0, 255), 1)
    cv2.line(out, (icr, ft), (icr, fb), (255, 0, 255), 1)
    cv2.putText(out, f"FL={FwL:.2f}", (fl + 4, ft + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    cv2.putText(out, f"FR={FwR:.2f}", (fmid + 4, ft + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # pre-corner box (magenta), x 0.15-0.85, y 0-150px
    py2 = min(h, PRECORNER_Y_BOTTOM_PX)
    px1 = int(PRECORNER_X_LEFT * w)
    px2 = int(PRECORNER_X_RIGHT * w)
    Pw = precorner_wall_amount(frame)
    cv2.rectangle(out, (px1, PRECORNER_Y_TOP_PX), (px2, py2), (255, 0, 255), 2)
    cv2.putText(out, f"P={Pw:.2f}", (px2 - 105, py2 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

    lt, lb = int(LINE_ROI_TOP * h), int(LINE_ROI_BOTTOM * h)
    ll, lr = int(LINE_ROI_LEFT * w), int(LINE_ROI_RIGHT * w)
    color = line_color(frame)
    box = (0, 165, 255) if color == "ORANGE" else (255, 0, 0) if color == "BLUE" else (0, 0, 255)
    cv2.rectangle(out, (ll, lt), (lr, lb), box, 2)
    cv2.putText(out, f"line:{color} {extra}", (5, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    return out


def race(camera, drive, led, start_button):
    drive.set_steering(STEER_CENTER)
    time.sleep(0.3)

    # ---- wait for the start button (GPIO 16), showing the FULL preview ----
    print(f"[INFO] Waiting for button on GPIO {START_BUTTON_GPIO} to START (preview live)...")
    while not start_button.is_pressed:
        frame = camera.get_frame()
        camera.show_overlay(annotate(frame, "WAITING - press button"))
        time.sleep(0.1)
    time.sleep(0.3)   # debounce / let go of the button
    print("[INFO] Started. Press the button again to STOP.")

    start_time = monotonic()

    def log(msg):
        print(f"[t={monotonic() - start_time:7.2f}s] {msg}")

    state = "STRAIGHT"
    turn_direction = None
    first_line_color = None        # color of the first corner line; the other color is ignored afterwards
    first_line_color = None        # set at corner 1; the other color is ignored for the rest of the run
    corner_count = 0
    approach_until = 0.0
    turn_start = 0.0
    line_blind_until = 0.0
    finish_at = None
    stuck_ref = None
    stuck_since = monotonic()
    reverse_start = 0.0
    recover_start = 0.0
    recover_steer = "CENTER"
    smoothed_steer = 0.0
    # what was interrupted by the front-wall backup, and how to resume it
    resume_state = "STRAIGHT"      # "STRAIGHT" or "TURNING_90"
    turn_remaining = 0.0           # seconds of turn left when interrupted
    corner_backup_start = 0.0      # pre-corner short backup timer
    front_wall_side = None         # LEFT / RIGHT / BOTH -- which half held the wall
    backup_duration = REVERSE_DURATION_S   # per-case backup time (0.4 / 0.7 / stuck default)

    try:
        while True:
            if start_button.is_pressed:
                log("Stop button pressed.")
                break

            now = monotonic()
            frame = camera.get_frame()
            camera.show_overlay(annotate(frame, f"c={corner_count} {state}"))

            # ---- line watch (only while STRAIGHT, outside cooldown) ----
            if state == "STRAIGHT" and now >= line_blind_until:
                color = line_color(frame)
                # after the first corner, the OTHER color is invisible to the robot
                if color is not None and first_line_color is not None and color != first_line_color:
                    color = None
                if color is not None:
                    if turn_direction is None:
                        first_line_color = color
                        turn_direction = (FIRST_ORANGE_MEANS if color == "ORANGE"
                                          else ("LEFT" if FIRST_ORANGE_MEANS == "RIGHT" else "RIGHT"))
                        log(f">>> First line {color} -> all turns {turn_direction} "
                            f"(only {color} lines will be detected now)")
                    corner_count += 1
                    log(f">>> Corner {corner_count}/{TOTAL_CORNERS} ({color})")
                    state = "APPROACHING"
                    approach_until = now + TURN_DELAY_S
                    line_blind_until = now + LINE_BLIND_AFTER_CORNER_S

            if state == "STRAIGHT":
                # front-wall check (halved): which side(s) see the wall?
                FwL, FwR = front_wall_halves(frame)
                left_hit  = FwL >= FRONT_WALL_THRESHOLD
                right_hit = FwR >= FRONT_WALL_THRESHOLD
                if left_hit or right_hit:
                    if left_hit and right_hit:
                        front_wall_side = "BOTH"
                        backup_duration = BACKUP_BOTH_S      # straight back, 0.7s
                    elif left_hit:
                        front_wall_side = "LEFT"
                        backup_duration = BACKUP_ONE_SIDE_S  # steer left, 0.4s
                    else:
                        front_wall_side = "RIGHT"
                        backup_duration = BACKUP_ONE_SIDE_S  # steer right, 0.4s
                    log(f">>> FRONT WALL {front_wall_side} (FL={FwL:.2f} FR={FwR:.2f}) "
                        f"-- backup {backup_duration}s, then centering")
                    resume_state = "STRAIGHT"
                    state = "REVERSING"; reverse_start = now; stuck_ref = None
                    time.sleep(LOOP_DT); continue

                # stuck detection
                if stuck_ref is None:
                    stuck_ref = frame.copy(); stuck_since = now
                else:
                    if frame_diff(frame, stuck_ref) >= STUCK_FRAME_DIFF_THRESHOLD:
                        stuck_ref = frame.copy(); stuck_since = now
                    elif now - stuck_since >= STUCK_TIME_S:
                        log(">>> STUCK -- reversing")
                        state = "REVERSING"; reverse_start = now; stuck_ref = None
                        time.sleep(LOOP_DT); continue

                # camera wall centering
                lroi, rroi = wall_rois(frame)
                Lw, Rw = wall_amount(lroi), wall_amount(rroi)
                error = Lw - Rw
                if abs(error) > CAM_STEER_DEADBAND:
                    target = clamp(CAM_STEER_GAIN * error, -CAM_MAX_OFFSET, CAM_MAX_OFFSET)
                    target = target if STEER_RIGHT > 0 else -target
                else:
                    target = STEER_CENTER
                smoothed_steer = (1 - STEER_SMOOTH_ALPHA) * smoothed_steer + STEER_SMOOTH_ALPHA * target
                drive.set_steering(smoothed_steer)
                drive.forward(STRAIGHT_SPEED)
                log(f"Lw={Lw:.2f} Rw={Rw:.2f} err={error:+.2f} steer={smoothed_steer:+.0f} c={corner_count}")

            elif state == "APPROACHING":
                # steer FULL LOCK toward the turn direction right away to clear the corner
                drive.set_steering(STEER_LEFT if turn_direction == "LEFT" else STEER_RIGHT)
                drive.forward(STRAIGHT_SPEED)
                if now >= approach_until:
                    # no stop before the turn: check the pre-corner box on-the-fly
                    Pw = precorner_wall_amount(frame)
                    if Pw >= PRECORNER_WALL_THRESHOLD:
                        log(f">>> wall in pre-corner box (P={Pw:.2f}) -- "
                            f"reversing before the turn")
                        drive.stop()
                        state = "CORNER_BACKUP"; corner_backup_start = now
                    else:
                        log(">>> turning (no stop)")
                        state = "TURNING_90"; turn_start = now

            elif state == "CORNER_BACKUP":
                # reverse UNTIL the pre-corner strip no longer sees a wall (capped)
                drive.set_steering(STEER_CENTER)
                drive.backward(REVERSE_SPEED)
                Pw = precorner_wall_amount(frame)
                if Pw < PRECORNER_WALL_THRESHOLD or now - corner_backup_start >= BACKUP_MAX_S:
                    drive.stop()
                    log(f">>> pre-corner clear (P={Pw:.2f}) -- turning")
                    state = "TURNING_90"; turn_start = now

            elif state == "TURNING_90":
                # front-wall check DURING the corner: back up (counter-steer), then
                # finish the remaining turn time
                Fw = front_wall_amount(frame)
                if Fw >= FRONT_WALL_THRESHOLD:
                    turn_remaining = max(0.2, TURN_90_DURATION_S - (now - turn_start))
                    log(f">>> FRONT WALL mid-corner (F={Fw:.2f}) -- backup, "
                        f"then finish turn ({turn_remaining:.2f}s left)")
                    resume_state = "TURNING_90"
                    state = "REVERSING"; reverse_start = now
                    time.sleep(LOOP_DT); continue

                drive.set_steering(STEER_LEFT if turn_direction == "LEFT" else STEER_RIGHT)
                drive.forward(TURN_SPEED)
                if now - turn_start >= TURN_90_DURATION_S:
                    drive.set_steering(STEER_CENTER)
                    line_blind_until = now + CORNER_COOLDOWN_S
                    if corner_count >= TOTAL_CORNERS:
                        state = "FINISHING"; finish_at = now + STOP_DELAY_S
                        log(">>> last corner done")
                    else:
                        state = "STRAIGHT"

            elif state == "REVERSING":
                if resume_state == "TURNING_90":
                    # corner backup: counter-steer, reverse UNTIL the front box is clear
                    drive.set_steering(STEER_RIGHT if turn_direction == "LEFT" else STEER_LEFT)
                    drive.backward(REVERSE_SPEED)
                    FwL, FwR = front_wall_halves(frame)
                    front_clear = (FwL + FwR) / 2.0 < FRONT_WALL_THRESHOLD
                    if front_clear or now - reverse_start >= BACKUP_MAX_S:
                        drive.stop()
                        line_blind_until = now + LINE_BLIND_AFTER_REVERSE_S
                        log(f">>> front clear (FL={FwL:.2f} FR={FwR:.2f}) -- "
                            f"resuming corner ({turn_remaining:.2f}s)")
                        turn_start = now - (TURN_90_DURATION_S - turn_remaining)
                        state = "TURNING_90"
                else:
                    # centering backup:
                    #   BOTH halves -> straight back (0.7s)
                    #   LEFT only  -> steer LEFT while backing (0.4s)
                    #   RIGHT only -> steer RIGHT while backing (0.4s)
                    if front_wall_side == "LEFT":
                        drive.set_steering(STEER_LEFT)
                    elif front_wall_side == "RIGHT":
                        drive.set_steering(STEER_RIGHT)
                    else:                       # BOTH, or stuck-recovery
                        drive.set_steering(STEER_CENTER)
                    drive.backward(REVERSE_SPEED)
                    if now - reverse_start >= backup_duration:
                        drive.stop()
                        line_blind_until = now + LINE_BLIND_AFTER_REVERSE_S
                        drive.set_steering(STEER_CENTER)
                        front_wall_side = None
                        log(">>> backup done -- steering centered, resuming centering")
                        state = "STRAIGHT"

            elif state == "RECOVER_FORWARD":
                drive.set_steering(STEER_LEFT if recover_steer == "LEFT"
                                   else STEER_RIGHT if recover_steer == "RIGHT" else STEER_CENTER)
                drive.forward(RECOVER_SPEED)
                if now - recover_start >= RECOVER_FORWARD_DURATION_S:
                    drive.set_steering(STEER_CENTER)
                    state = "STRAIGHT"

            elif state == "FINISHING":
                drive.set_steering(STEER_CENTER)
                drive.forward(STRAIGHT_SPEED)
                if now >= finish_at:
                    log(">>> 3 laps complete. Stopping.")
                    drive.stop()
                    led.on()
                    break

            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        log("manual stop")
    finally:
        drive.stop()
        drive.set_steering(STEER_CENTER)
        drive.cleanup()
        camera.stop()


def preview(camera):
    print("preview (no motors) -- Ctrl+C to quit")
    try:
        while True:
            frame = camera.get_frame()
            camera.show_overlay(annotate(frame))
            lroi, rroi = wall_rois(frame)
            print(f"Lw={wall_amount(lroi):.2f} Rw={wall_amount(rroi):.2f} line={line_color(frame)}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()


def snapshot(camera):
    frame = camera.get_frame()
    lroi, rroi = wall_rois(frame)
    print(f"Lw={wall_amount(lroi):.2f} Rw={wall_amount(rroi):.2f} line={line_color(frame)}")
    cv2.imwrite("race_snapshot.png", annotate(frame))
    print("saved race_snapshot.png")
    camera.stop()

if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "race"
    camera = Camera(); camera.start()
    if mode == "preview":
        preview(camera)
    elif mode == "snapshot":
        snapshot(camera)
    else:
        drive = Drive()
        led = LED(LED_PIN); led.off()
        start_button = Button(START_BUTTON_GPIO, pull_up=False, bounce_time=0.08)
        race(camera, drive, led, start_button)
