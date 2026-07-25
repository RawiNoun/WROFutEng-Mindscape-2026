#!/usr/bin/env python3
"""
obstacle.py  --  WRO Obstacle Challenge, full run.

Corner method (same as race.py):
  STRAIGHT -> line seen -> APPROACHING -> TURNING_90 -> STRAIGHT
  First line color sets direction for the whole run.
  12 corners = 3 laps -> FINISHING -> stop (+ LED).

Pillar avoidance (priority over wall centering while in STRAIGHT):
  RED  pillar -> pass on the RIGHT  (cx_frac <= 0.38 = passed)
  GREEN pillar -> pass on the LEFT  (cx_frac >= 0.62 = passed)
  Pillars take full priority: wall centering is suppressed while any pillar
  is visible. Corner detection still fires normally.

Wrong-side recovery (pillar):
  RED:   steer LEFT  + reverse 0.8s -> steer RIGHT + forward until passed
  GREEN: steer RIGHT + reverse 1.0s -> steer LEFT  + forward until passed

Modes:
    python3 obstacle.py           # race
    python3 obstacle.py preview   # vision only, no motors
    python3 obstacle.py snapshot  # save one annotated frame
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
TURN_DELAY_S        = 0.3
TURN_90_DURATION_S  = 0.7
TURN_SPEED          = 0.25
STOP_BEFORE_TURN_S  = 0.10
CORNER_COOLDOWN_S   = 0.3
FIRST_ORANGE_MEANS  = "RIGHT"

# ════════════════ DRIVING ════════════════
STRAIGHT_SPEED      = 0.25
PILLAR_FOLLOW_SPEED = 0.20   # forward speed whenever a pillar is visible
REVERSE_SPEED       = 0.3
RECOVER_SPEED       = 0.25
STEER_CENTER = 0
STEER_LEFT   = -70
STEER_RIGHT  =  70

# ── wall centering ────────────────────────────────────────────────────────────
WALL_ROI_TOP       = 0.15
WALL_ROI_BOTTOM    = 0.40
WALL_ROI_WIDTH     = 0.20
WALL_L_MAX         = 110
CAM_STEER_DEADBAND = 0.06
CAM_STEER_GAIN     = 90
CAM_MAX_OFFSET     = 70

# Separate smoothing constants (from test_obstacle)
PILLAR_SMOOTH_ALPHA = 0.15   # snappy correction while pillar active
WALL_SMOOTH_ALPHA   = 0.35   # gentle cruising feel

# ── front wall ────────────────────────────────────────────────────────────────
FRONT_ROI_TOP        = 0.30
FRONT_ROI_BOTTOM     = 0.65
FRONT_ROI_LEFT       = 0.25
FRONT_ROI_RIGHT      = 0.75
FRONT_WALL_THRESHOLD = 0.30
BACKUP_ONE_SIDE_S    = 0.4
BACKUP_BOTH_S        = 0.7
LINE_BLIND_AFTER_REVERSE_S = 1.0
LINE_BLIND_AFTER_CORNER_S  = 2.0

# ── pre-corner danger box ─────────────────────────────────────────────────────
PRECORNER_Y_TOP_PX       = 80
PRECORNER_Y_BOTTOM_PX    = 120
PRECORNER_X_LEFT         = FRONT_ROI_LEFT  + 0.15
PRECORNER_X_RIGHT        = FRONT_ROI_RIGHT - 0.15
PRECORNER_WALL_THRESHOLD = 0.30
PRECORNER_REVERSE_S      = 0.2
BACKUP_MAX_S             = 0.4
CORNER_WAIT_S            = 1.0

# ── stuck detection ───────────────────────────────────────────────────────────
STUCK_FRAME_DIFF_THRESHOLD = 5.0
STUCK_TIME_S               = 1
REVERSE_DURATION_S         = 1
RECOVER_FORWARD_DURATION_S = 0.2

# ── line detection ────────────────────────────────────────────────────────────
LINE_ROI_TOP    = 0.85
LINE_ROI_BOTTOM = 1.0
LINE_ROI_LEFT   = 0.30
LINE_ROI_RIGHT  = 0.70
ORANGE_LO = np.array([  5, 100,  80]);  ORANGE_HI = np.array([ 25, 255, 255])
BLUE_LO   = np.array([ 90,  80,  40]);  BLUE_HI   = np.array([130, 255, 255])
LINE_MIN_PIXELS = 150

# ── pillar detection ──────────────────────────────────────────────────────────
LOWER_RED1  = np.array([  0, 120,  70])
UPPER_RED1  = np.array([ 10, 255, 255])
LOWER_RED2  = np.array([170, 120,  70])
UPPER_RED2  = np.array([180, 255, 255])
LOWER_GREEN = np.array([ 40,  70,  50])
UPPER_GREEN = np.array([ 85, 255, 255])

PILLAR_ROI_TOP    = 0.10
PILLAR_ROI_BOTTOM = 0.75
PILLAR_ROI_LEFT   = 0.05
PILLAR_ROI_RIGHT  = 0.95
PILLAR_MIN_AREA   = 700

# RED: pass on the RIGHT
PILLAR_RED_PASSED_X = 0.3
PILLAR_RED_WRONG_X  = PILLAR_RED_PASSED_X 

# GREEN: pass on the LEFT
PILLAR_GREEN_PASSED_X = 0.7
PILLAR_GREEN_WRONG_X  = PILLAR_GREEN_PASSED_X 

# Pass corridor: the path between the two PASS lines
PASS_PATH_LEFT      = 0.4    # corridor edges (independent of the PASS lines)
PASS_PATH_RIGHT     = 0.6
PASS_PATH_TOP       = 0.30   # vertical span of the corridor check
PASS_PATH_BOTTOM    = 0.65
PASS_PATH_THRESHOLD = 0.05   # wall amount below this = corridor empty
PASS_PATH_HOLD_S    = 0.5    # keep going straight this long after pillar is gone

# Pillar too close -> stop and reverse straight
PILLAR_TOO_CLOSE_FRAC      = 0.25   # pillar area >= 25% of the whole frame
PILLAR_TOO_CLOSE_REVERSE_S = 0.8

# Wrong-side recovery durations
RED_WRONG_SIDE_REVERSE_S          = 0.5
RED_DANGER_WRONG_SIDE_REVERSE_S   = RED_WRONG_SIDE_REVERSE_S / 2
GREEN_WRONG_SIDE_REVERSE_S        = 1.0
GREEN_DANGER_WRONG_SIDE_REVERSE_S = GREEN_WRONG_SIDE_REVERSE_S / 2

TOTAL_CORNERS = 12
STOP_DELAY_S  = 0.5
LOOP_DT       = 0.06

LED_PIN           = 22
START_BUTTON_GPIO = 16


# ══════════════════════════════════════════════════════════════════════════════
#  HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wall_rois(frame):
    h, w = frame.shape[:2]
    t, b = int(WALL_ROI_TOP * h), int(WALL_ROI_BOTTOM * h)
    ww   = int(WALL_ROI_WIDTH * w)
    return frame[t:b, 0:ww], frame[t:b, w - ww:w]


def wall_amount(roi_bgr):
    lab  = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2LAB)
    mask = lab[:, :, 0] < WALL_L_MAX
    return float(np.count_nonzero(mask)) / mask.size


def front_wall_amount(frame):
    h, w = frame.shape[:2]
    t, b = int(FRONT_ROI_TOP * h), int(FRONT_ROI_BOTTOM * h)
    l, r = int(FRONT_ROI_LEFT * w), int(FRONT_ROI_RIGHT * w)
    lab  = cv2.cvtColor(frame[t:b, l:r], cv2.COLOR_BGR2LAB)
    mask = lab[:, :, 0] < WALL_L_MAX
    return float(np.count_nonzero(mask)) / mask.size


def front_wall_halves(frame):
    h, w = frame.shape[:2]
    t, b = int(FRONT_ROI_TOP * h), int(FRONT_ROI_BOTTOM * h)
    l, r = int(FRONT_ROI_LEFT * w), int(FRONT_ROI_RIGHT * w)
    mid  = (l + r) // 2
    lab_l = cv2.cvtColor(frame[t:b, l:mid], cv2.COLOR_BGR2LAB)
    lab_r = cv2.cvtColor(frame[t:b, mid:r], cv2.COLOR_BGR2LAB)
    ml = lab_l[:, :, 0] < WALL_L_MAX
    mr = lab_r[:, :, 0] < WALL_L_MAX
    return (float(np.count_nonzero(ml)) / ml.size,
            float(np.count_nonzero(mr)) / mr.size)


def precorner_wall_amount(frame):
    h, w = frame.shape[:2]
    y1 = max(0, PRECORNER_Y_TOP_PX)
    y2 = min(h, PRECORNER_Y_BOTTOM_PX)
    x1 = int(PRECORNER_X_LEFT  * w)
    x2 = int(PRECORNER_X_RIGHT * w)
    lab  = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2LAB)
    mask = lab[:, :, 0] < WALL_L_MAX
    return float(np.count_nonzero(mask)) / mask.size


def pass_path_amount(frame):
    """Wall amount inside the corridor between the two PASS lines."""
    h, w = frame.shape[:2]
    t, b = int(PASS_PATH_TOP * h), int(PASS_PATH_BOTTOM * h)
    l, r = int(PASS_PATH_LEFT * w), int(PASS_PATH_RIGHT * w)
    lab  = cv2.cvtColor(frame[t:b, l:r], cv2.COLOR_BGR2LAB)
    mask = lab[:, :, 0] < WALL_L_MAX
    return float(np.count_nonzero(mask)) / mask.size


def pillar_in_danger_box(detection, frame_shape):
    if detection is None:
        return False
    h, w = frame_shape[:2]
    y1 = max(0, PRECORNER_Y_TOP_PX)
    y2 = min(h, PRECORNER_Y_BOTTOM_PX)
    x1 = int(PRECORNER_X_LEFT  * w)
    x2 = int(PRECORNER_X_RIGHT * w)
    cx, cy, area = detection
    return (x1 <= cx <= x2) and (y1 <= cy <= y2)


def pillar_area_frac(detection, frame_shape):
    """Pillar contour area as a fraction of the whole frame."""
    if detection is None:
        return 0.0
    h, w = frame_shape[:2]
    return float(detection[2]) / float(h * w)


def line_color(frame):
    h, w = frame.shape[:2]
    t, b = int(LINE_ROI_TOP * h), int(LINE_ROI_BOTTOM * h)
    l, r = int(LINE_ROI_LEFT * w), int(LINE_ROI_RIGHT * w)
    hsv    = cv2.cvtColor(frame[t:b, l:r], cv2.COLOR_BGR2HSV)
    orange = cv2.countNonZero(cv2.inRange(hsv, ORANGE_LO, ORANGE_HI))
    blue   = cv2.countNonZero(cv2.inRange(hsv, BLUE_LO,   BLUE_HI))
    if orange >= LINE_MIN_PIXELS and orange >= blue:
        return "ORANGE"
    if blue >= LINE_MIN_PIXELS:
        return "BLUE"
    return None


def frame_diff(a, b):
    ga = cv2.resize(cv2.cvtColor(a, cv2.COLOR_BGR2GRAY), (80, 60))
    gb = cv2.resize(cv2.cvtColor(b, cv2.COLOR_BGR2GRAY), (80, 60))
    return cv2.absdiff(ga, gb).mean()


def detect_red(frame):
    """Detect the largest RED pillar. Returns (cx, cy, area) or None."""
    h, w = frame.shape[:2]
    y1 = int(PILLAR_ROI_TOP    * h)
    y2 = int(PILLAR_ROI_BOTTOM * h)
    x1 = int(PILLAR_ROI_LEFT   * w)
    x2 = int(PILLAR_ROI_RIGHT  * w)
    roi    = frame[y1:y2, x1:x2]
    hsv    = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask_r = cv2.bitwise_or(
        cv2.inRange(hsv, LOWER_RED1, UPPER_RED1),
        cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    )
    contours, _ = cv2.findContours(mask_r, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < PILLAR_MIN_AREA:
            continue
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"]) + x1
        cy = int(M["m01"] / M["m00"]) + y1
        if best is None or area > best[2]:
            best = (cx, cy, area)
    return best


def detect_green(frame):
    """Detect the largest GREEN pillar. Returns (cx, cy, area) or None."""
    h, w = frame.shape[:2]
    y1 = int(PILLAR_ROI_TOP    * h)
    y2 = int(PILLAR_ROI_BOTTOM * h)
    x1 = int(PILLAR_ROI_LEFT   * w)
    x2 = int(PILLAR_ROI_RIGHT  * w)
    roi    = frame[y1:y2, x1:x2]
    hsv    = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask_g = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
    contours, _ = cv2.findContours(mask_g, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < PILLAR_MIN_AREA:
            continue
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"]) + x1
        cy = int(M["m01"] / M["m00"]) + y1
        if best is None or area > best[2]:
            best = (cx, cy, area)
    return best


def annotate(frame, det_red, det_green, extra=""):
    out = frame.copy()
    h, w = out.shape[:2]

    # Wall ROI boxes
    t, b = int(WALL_ROI_TOP * h), int(WALL_ROI_BOTTOM * h)
    ww   = int(WALL_ROI_WIDTH * w)
    lroi, rroi = wall_rois(frame)
    Lw, Rw = wall_amount(lroi), wall_amount(rroi)
    cv2.rectangle(out, (0, t),      (ww, b), (0, 255, 0), 2)
    cv2.rectangle(out, (w - ww, t), (w,  b), (0, 255, 0), 2)
    cv2.putText(out, f"{Lw:.2f}", (4, t + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(out, f"{Rw:.2f}", (w - ww + 4, t + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Front wall box (yellow), split into halves
    ft, fb = int(FRONT_ROI_TOP * h), int(FRONT_ROI_BOTTOM * h)
    fl, fr = int(FRONT_ROI_LEFT * w), int(FRONT_ROI_RIGHT * w)
    fmid   = (fl + fr) // 2
    FwL, FwR = front_wall_halves(frame)
    cv2.rectangle(out, (fl, ft), (fr, fb), (0, 255, 255), 2)
    cv2.line(out, (fmid, ft), (fmid, fb), (0, 255, 255), 1)
    icl = int(PRECORNER_X_LEFT  * w)
    icr = int(PRECORNER_X_RIGHT * w)
    cv2.line(out, (icl, ft), (icl, fb), (255, 0, 255), 1)
    cv2.line(out, (icr, ft), (icr, fb), (255, 0, 255), 1)
    cv2.putText(out, f"FL={FwL:.2f}", (fl + 4, ft + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    cv2.putText(out, f"FR={FwR:.2f}", (fmid + 4, ft + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # Pass corridor (yellow)
    ppt, ppb = int(PASS_PATH_TOP * h), int(PASS_PATH_BOTTOM * h)
    ppl, ppr = int(PASS_PATH_LEFT * w), int(PASS_PATH_RIGHT * w)
    ppv = pass_path_amount(frame)
    cv2.rectangle(out, (ppl, ppt), (ppr, ppb), (0, 255, 255), 2)
    cv2.putText(out, f"pp={ppv:.2f}", (ppl + 4, ppb - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # Pre-corner box (magenta)
    py2 = min(h, PRECORNER_Y_BOTTOM_PX)
    px1 = int(PRECORNER_X_LEFT  * w)
    px2 = int(PRECORNER_X_RIGHT * w)
    Pw  = precorner_wall_amount(frame)
    cv2.rectangle(out, (px1, PRECORNER_Y_TOP_PX), (px2, py2), (255, 0, 255), 2)
    cv2.putText(out, f"P={Pw:.2f}", (px2 - 105, py2 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

    # Pillar-in-danger-box indicator: 1/3 smaller magenta box, drawn when a pillar centroid is inside
    db_cx  = (px1 + px2) // 2
    db_cy  = (PRECORNER_Y_TOP_PX + py2) // 2
    db_hw  = (px2 - px1) // 3        # half-width of smaller box
    db_hh  = (py2 - PRECORNER_Y_TOP_PX) // 3   # half-height of smaller box
    active_det = det_red if det_red else det_green
    if active_det and pillar_in_danger_box(active_det, frame.shape):
        cv2.rectangle(out,
                      (db_cx - db_hw, db_cy - db_hh),
                      (db_cx + db_hw, db_cy + db_hh),
                      (255, 0, 255), 3)

    # Line ROI box
    lt, lb = int(LINE_ROI_TOP * h), int(LINE_ROI_BOTTOM * h)
    ll, lr = int(LINE_ROI_LEFT * w), int(LINE_ROI_RIGHT * w)
    color  = line_color(frame)
    box    = (0, 165, 255) if color == "ORANGE" else (255, 0, 0) if color == "BLUE" else (0, 0, 255)
    cv2.rectangle(out, (ll, lt), (lr, lb), box, 2)

    # Pillar ROI boundary (white)
    pr_t = int(PILLAR_ROI_TOP    * h)
    pr_b = int(PILLAR_ROI_BOTTOM * h)
    pr_l = int(PILLAR_ROI_LEFT   * w)
    pr_r = int(PILLAR_ROI_RIGHT  * w)
    cv2.rectangle(out, (pr_l, pr_t), (pr_r, pr_b), (200, 200, 200), 1)

    # RED threshold lines
    rx  = int(PILLAR_RED_PASSED_X * w)
    rwx = int(PILLAR_RED_WRONG_X  * w)
    cv2.line(out, (rx,  pr_t), (rx,  pr_b), (0, 0, 220), 2)
    cv2.putText(out, "<PASS", (rx + 4, pr_t + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 220), 1)
    cv2.line(out, (rwx, pr_t), (rwx, pr_b), (0, 0, 255), 2)
    cv2.putText(out, "WRONG>", (rwx + 4, pr_t + 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

    # GREEN threshold lines
    gpx = int(PILLAR_GREEN_PASSED_X * w)
    gwx = int(PILLAR_GREEN_WRONG_X  * w)
    cv2.line(out, (gpx, pr_t), (gpx, pr_b), (0, 200, 0), 2)
    cv2.putText(out, "PASS>", (gpx + 4, pr_t + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 0), 1)
    cv2.line(out, (gwx, pr_t), (gwx, pr_b), (0, 128, 0), 2)
    cv2.putText(out, "<WRONG", (gwx - 65, pr_t + 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 128, 0), 1)

    # RED pillar dot + label
    if det_red:
        cx, cy, area = det_red
        cx_frac = cx / w
        if cx_frac <= PILLAR_RED_PASSED_X:
            label, col = "PASSED", (0, 255, 255)
        elif cx_frac > PILLAR_RED_WRONG_X:
            label, col = "WRONG-SIDE", (0, 0, 255)
        else:
            label, col = "STEER-R", (0, 0, 255)
        cv2.circle(out, (cx, cy), 10, (0, 0, 255), -1)
        cv2.circle(out, (cx, cy), 12, (255, 255, 255), 2)
        cv2.line(out, (cx, pr_t), (cx, pr_b), (0, 0, 200), 1)
        cv2.putText(out, f"RED {label} a={int(area)} cx={cx_frac:.2f}",
                    (cx + 14, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)

    # GREEN pillar dot + label
    if det_green:
        cx, cy, area = det_green
        cx_frac = cx / w
        if cx_frac >= PILLAR_GREEN_PASSED_X:
            label, col = "PASSED", (0, 255, 100)
        elif cx_frac < PILLAR_GREEN_WRONG_X:
            label, col = "WRONG-SIDE", (0, 180, 0)
        else:
            label, col = "STEER-L", (0, 255, 0)
        cv2.circle(out, (cx, cy), 10, (0, 200, 0), -1)
        cv2.circle(out, (cx, cy), 12, (255, 255, 255), 2)
        cv2.line(out, (cx, pr_t), (cx, pr_b), (0, 180, 0), 1)
        cv2.putText(out, f"GRN {label} a={int(area)} cx={cx_frac:.2f}",
                    (cx + 14, cy + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)

    cv2.putText(out, f"line:{color} {extra}", (5, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    return out


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN RACE LOOP
# ══════════════════════════════════════════════════════════════════════════════

def race(camera, drive, led, start_button):
    drive.set_steering(STEER_CENTER)
    time.sleep(0.3)

    print(f"[INFO] Waiting for button on GPIO {START_BUTTON_GPIO} to START...")
    while not start_button.is_pressed:
        frame     = camera.get_frame()
        det_red   = detect_red(frame)
        det_green = detect_green(frame)
        led.on()
        camera.show_overlay(annotate(frame, det_red, det_green, "WAITING - press button"))
        time.sleep(0.1)
    time.sleep(0.3)
    print("[INFO] Started. Press the button again to STOP.")

    start_time = monotonic()

    def log(msg):
        print(f"[t={monotonic() - start_time:7.2f}s] {msg}")

    # ── race.py state variables ───────────────────────────────────────────────
    state            = "STRAIGHT"
    turn_direction   = None
    first_line_color = None
    corner_count     = 0
    approach_until   = 0.0
    turn_start       = 0.0
    line_blind_until = 0.0
    finish_at        = None
    stuck_ref        = None
    stuck_since      = monotonic()
    reverse_start    = 0.0
    recover_start    = 0.0
    recover_steer    = "CENTER"
    smoothed_steer   = 0.0
    resume_state     = "STRAIGHT"
    turn_remaining   = 0.0
    corner_backup_start = 0.0
    front_wall_side  = None
    backup_duration  = REVERSE_DURATION_S

    # ── pillar state variables ────────────────────────────────────────────────
    active_color           = None   # "RED" or "GREEN"
    passed_color           = None   # locked when a pillar enters passed zone
    last_steer             = STEER_CENTER  # last steering direction used
    wrong_side_start       = 0.0
    too_close_start        = 0.0
    pass_path_until        = 0.0
    danger_triggered       = False
    reapproach_lost_frames = 0
    blind_until            = 0.0   # pillar-specific blind window after backup

    try:
        while True:
            if start_button.is_pressed:
                log("Stop button pressed.")
                break

            now       = monotonic()
            frame     = camera.get_frame()
            h, w      = frame.shape[:2]
            det_red   = detect_red(frame)
            det_green = detect_green(frame)

            # Active pillar: if one is in the passed zone, stay locked on it.
            # Otherwise closest wins every frame (larger area = closer).
            if passed_color == "RED":
                detection = det_red;   color = "RED" if det_red else None
                if not det_red:        passed_color = None   # gone from view, release
            elif passed_color == "GREEN":
                detection = det_green; color = "GREEN" if det_green else None
                if not det_green:      passed_color = None   # gone from view, release
            elif det_red and det_green:
                detection = det_red if det_red[2] >= det_green[2] else det_green
                color     = "RED"   if det_red[2] >= det_green[2] else "GREEN"
            elif det_red:
                detection = det_red;   color = "RED"
            elif det_green:
                detection = det_green; color = "GREEN"
            else:
                detection = None;      color = None

            camera.show_overlay(annotate(frame, det_red, det_green,
                                         f"c={corner_count} {state}"))

            # ── line watch (only while STRAIGHT, outside cooldown) ────────────
            if state in ("STRAIGHT", "PASS_PATH") and now >= line_blind_until:
                lc = line_color(frame)
                if lc is not None and first_line_color is not None and lc != first_line_color:
                    lc = None
                if lc is not None:
                    if turn_direction is None:
                        first_line_color = lc
                        turn_direction = (FIRST_ORANGE_MEANS if lc == "ORANGE"
                                          else ("LEFT" if FIRST_ORANGE_MEANS == "RIGHT" else "RIGHT"))
                        log(f">>> First line {lc} -> all turns {turn_direction}")
                    corner_count += 1
                    log(f">>> Corner {corner_count}/{TOTAL_CORNERS} ({lc})")
                    state = "APPROACHING"
                    approach_until   = now + TURN_DELAY_S
                    line_blind_until = now + LINE_BLIND_AFTER_CORNER_S

            # ══════════════════════════════════════════════════════════════════
            #  STRAIGHT  --  pillars have priority, corners still fire above
            # ══════════════════════════════════════════════════════════════════
            if state == "STRAIGHT":
                FwL, FwR = front_wall_halves(frame)
                Pw       = precorner_wall_amount(frame)

                # 0. Pillar too close -> stop + reverse straight
                too_close_frac = max(pillar_area_frac(det_red,   frame.shape),
                                     pillar_area_frac(det_green, frame.shape))
                if too_close_frac >= PILLAR_TOO_CLOSE_FRAC:
                    log(f"PILLAR TOO CLOSE ({too_close_frac:.2f} of frame) -> stop + reverse straight")
                    drive.stop()
                    state = "PILLAR_TOO_CLOSE"; too_close_start = now
                    time.sleep(LOOP_DT); continue

                # 1. Pre-corner danger box check
                if Pw >= PRECORNER_WALL_THRESHOLD and now >= blind_until:
                    if pillar_in_danger_box(detection, frame.shape):
                        log(f"DANGER BOX {color} P={Pw:.2f} -> wrong-side recovery (halved)")
                        active_color     = color
                        state            = "PILLAR_WRONG_SIDE"
                        wrong_side_start = now
                        danger_triggered = True
                        time.sleep(LOOP_DT); continue
                    else:
                        log(f"DANGER BOX WALL P={Pw:.2f} -> reversing")
                        drive.stop()
                        resume_state    = "STRAIGHT"
                        front_wall_side = None
                        backup_duration = BACKUP_BOTH_S
                        state           = "REVERSING"
                        reverse_start   = now
                        time.sleep(LOOP_DT); continue

                # 2. Front wall check
                left_hit  = FwL >= FRONT_WALL_THRESHOLD
                right_hit = FwR >= FRONT_WALL_THRESHOLD
                if (left_hit or right_hit) and now >= blind_until:
                    if left_hit and right_hit:
                        front_wall_side = "BOTH"
                        backup_duration = BACKUP_BOTH_S
                    elif left_hit:
                        front_wall_side = "LEFT"
                        backup_duration = BACKUP_ONE_SIDE_S
                    else:
                        front_wall_side = "RIGHT"
                        backup_duration = BACKUP_ONE_SIDE_S
                    log(f"FRONT WALL {front_wall_side} FL={FwL:.2f} FR={FwR:.2f}")
                    resume_state = "STRAIGHT"
                    state = "REVERSING"; reverse_start = now; stuck_ref = None
                    time.sleep(LOOP_DT); continue

                # 3. Stuck detection
                if stuck_ref is None:
                    stuck_ref = frame.copy(); stuck_since = now
                else:
                    if frame_diff(frame, stuck_ref) >= STUCK_FRAME_DIFF_THRESHOLD:
                        stuck_ref = frame.copy(); stuck_since = now
                    elif now - stuck_since >= STUCK_TIME_S:
                        log("STUCK -> reversing")
                        front_wall_side = None
                        backup_duration = REVERSE_DURATION_S
                        resume_state = "STRAIGHT"
                        state = "REVERSING"; reverse_start = now; stuck_ref = None
                        time.sleep(LOOP_DT); continue

                # 4. Pillar logic (priority over wall centering)
                if detection and now >= blind_until:
                    cx, cy, area = detection
                    cx_frac = cx / w

                    if color == "RED":
                        if cx_frac > PILLAR_RED_WRONG_X:
                            log(f"RED WRONG SIDE cx={cx_frac:.2f} -> recovering")
                            active_color     = "RED"
                            state            = "PILLAR_WRONG_SIDE"
                            wrong_side_start = now
                            danger_triggered = False
                            time.sleep(LOOP_DT); continue
                        passed = cx_frac <= PILLAR_RED_PASSED_X
                        if not passed:
                            target = float(STEER_RIGHT)
                            alpha  = PILLAR_SMOOTH_ALPHA
                            log(f"RED STEER-R cx={cx_frac:.2f} steer={target:+.0f}")
                        else:
                            ppw = pass_path_amount(frame)
                            if ppw < PASS_PATH_THRESHOLD:
                                log(f"RED PASSED + path clear (pp={ppw:.2f}) -> PASS_PATH cx={cx_frac:.2f}")
                                active_color    = "RED"
                                passed_color    = "RED"
                                pass_path_until = now + PASS_PATH_HOLD_S
                                state = "PASS_PATH"
                                time.sleep(LOOP_DT); continue
                            target = STEER_CENTER
                            alpha  = PILLAR_SMOOTH_ALPHA
                            passed_color = "RED"
                            log(f"RED PASSED (still visible) -> straight cx={cx_frac:.2f} pp={ppw:.2f}")

                    else:  # GREEN
                        if cx_frac < PILLAR_GREEN_WRONG_X:
                            log(f"GREEN WRONG SIDE cx={cx_frac:.2f} -> recovering")
                            active_color     = "GREEN"
                            state            = "PILLAR_WRONG_SIDE"
                            wrong_side_start = now
                            danger_triggered = False
                            time.sleep(LOOP_DT); continue
                        passed = cx_frac >= PILLAR_GREEN_PASSED_X
                        if not passed:
                            target = float(STEER_LEFT)
                            alpha  = PILLAR_SMOOTH_ALPHA
                            log(f"GRN STEER-L cx={cx_frac:.2f} steer={target:+.0f}")
                        else:
                            ppw = pass_path_amount(frame)
                            if ppw < PASS_PATH_THRESHOLD:
                                log(f"GRN PASSED + path clear (pp={ppw:.2f}) -> PASS_PATH cx={cx_frac:.2f}")
                                active_color    = "GREEN"
                                passed_color    = "GREEN"
                                pass_path_until = now + PASS_PATH_HOLD_S
                                state = "PASS_PATH"
                                time.sleep(LOOP_DT); continue
                            target = STEER_CENTER
                            alpha  = PILLAR_SMOOTH_ALPHA
                            passed_color = "GREEN"
                            log(f"GRN PASSED (still visible) -> straight cx={cx_frac:.2f} pp={ppw:.2f}")

                else:
                    # 5. No pillar -> wall centering or last_steer fallback
                    lroi, rroi = wall_rois(frame)
                    Lw, Rw = wall_amount(lroi), wall_amount(rroi)
                    left_wall  = Lw > CAM_STEER_DEADBAND
                    right_wall = Rw > CAM_STEER_DEADBAND
                    error = Lw - Rw

                    if left_wall and right_wall:
                        # both walls visible -> normal centering
                        if abs(error) > CAM_STEER_DEADBAND:
                            target = clamp(CAM_STEER_GAIN * error, -CAM_MAX_OFFSET, CAM_MAX_OFFSET)
                        else:
                            target = STEER_CENTER
                        log(f"WALL CENTER Lw={Lw:.2f} Rw={Rw:.2f} steer={target:+.0f} c={corner_count}")
                    elif not left_wall and not right_wall:
                        # no pillar AND no walls -> steer opposite of last_steer
                        target = -last_steer if last_steer != STEER_CENTER else STEER_CENTER
                        log(f"NO PILLAR AND NO WALLS -> opposite of last_steer={last_steer:+.0f} target={target:+.0f}")
                    else:
                        # no pillar AND only one wall -> steer opposite of last_steer
                        target = -last_steer if last_steer != STEER_CENTER else STEER_CENTER
                        log(f"NO PILLAR AND ONE WALL Lw={Lw:.2f} Rw={Rw:.2f} -> opposite last_steer={target:+.0f}")
                    alpha = WALL_SMOOTH_ALPHA

                smoothed_steer = (1 - alpha) * smoothed_steer + alpha * target
                last_steer     = smoothed_steer   # record for fallback
                drive.set_steering(smoothed_steer)   # steering FIRST
                fwd_speed = PILLAR_FOLLOW_SPEED if (detection and now >= blind_until) else STRAIGHT_SPEED
                drive.forward(fwd_speed)

            # ══════════════════════════════════════════════════════════════════
            #  PASS_PATH: pillar is on its pass side and the corridor between
            #             the two PASS lines is empty -> straight forward
            # ══════════════════════════════════════════════════════════════════
            elif state == "PASS_PATH":
                drive.set_steering(STEER_CENTER)   # steering FIRST
                drive.forward(STRAIGHT_SPEED)
                pp_det = det_red if active_color == "RED" else det_green
                if pp_det:
                    pass_path_until = now + PASS_PATH_HOLD_S
                    log(f"PASS_PATH {active_color} straight (pillar visible)")
                else:
                    log(f"PASS_PATH {active_color} straight ({pass_path_until - now:.2f}s left)")
                    if now >= pass_path_until:
                        smoothed_steer = 0.0
                        passed_color   = None
                        log("PASS_PATH done -> STRAIGHT")
                        state = "STRAIGHT"

            # ══════════════════════════════════════════════════════════════════
            #  PILLAR_TOO_CLOSE: pillar fills >=25% of frame -> reverse straight
            # ══════════════════════════════════════════════════════════════════
            elif state == "PILLAR_TOO_CLOSE":
                drive.set_steering(STEER_CENTER)   # steering FIRST
                drive.backward(REVERSE_SPEED)
                log(f"TOO-CLOSE reverse straight t={now - too_close_start:.2f}s")
                if now - too_close_start >= PILLAR_TOO_CLOSE_REVERSE_S:
                    drive.stop()
                    smoothed_steer = 0.0
                    stuck_ref      = None
                    log("TOO-CLOSE reverse done -> STRAIGHT")
                    state = "STRAIGHT"

            # ══════════════════════════════════════════════════════════════════
            #  PILLAR_WRONG_SIDE: steer OPPOSITE + reverse
            # ══════════════════════════════════════════════════════════════════
            elif state == "PILLAR_WRONG_SIDE":
                if active_color == "RED":
                    ws_dur = RED_DANGER_WRONG_SIDE_REVERSE_S if danger_triggered \
                             else RED_WRONG_SIDE_REVERSE_S
                    drive.set_steering(STEER_LEFT)   # steering FIRST
                    drive.backward(REVERSE_SPEED)
                    log(f"RED WRONG-SIDE reverse+steer-L t={now - wrong_side_start:.2f}s"
                        f"{' (halved)' if danger_triggered else ''}")
                else:  # GREEN
                    ws_dur = GREEN_DANGER_WRONG_SIDE_REVERSE_S if danger_triggered \
                             else GREEN_WRONG_SIDE_REVERSE_S
                    drive.set_steering(STEER_RIGHT)  # steering FIRST
                    drive.backward(REVERSE_SPEED)
                    log(f"GRN WRONG-SIDE reverse+steer-R t={now - wrong_side_start:.2f}s"
                        f"{' (halved)' if danger_triggered else ''}")

                if now - wrong_side_start >= ws_dur:
                    drive.stop()
                    smoothed_steer         = 0.0
                    reapproach_lost_frames = 0
                    log("WRONG-SIDE reverse done -> PILLAR_REAPPROACH")
                    state = "PILLAR_REAPPROACH"

            # ══════════════════════════════════════════════════════════════════
            #  PILLAR_REAPPROACH: steer CORRECT + forward until passed
            # ══════════════════════════════════════════════════════════════════
            elif state == "PILLAR_REAPPROACH":
                if active_color == "RED":
                    drive.set_steering(STEER_RIGHT)   # steering FIRST
                    drive.forward(PILLAR_FOLLOW_SPEED)
                    smoothed_steer       = float(STEER_RIGHT)
                    reapproach_det       = det_red
                    passed_check         = lambda cf: cf <= PILLAR_RED_PASSED_X
                    steer_label          = "steer-R fwd"
                else:  # GREEN
                    drive.set_steering(STEER_LEFT)    # steering FIRST
                    drive.forward(PILLAR_FOLLOW_SPEED)
                    smoothed_steer       = float(STEER_LEFT)
                    reapproach_det       = det_green
                    passed_check         = lambda cf: cf >= PILLAR_GREEN_PASSED_X
                    steer_label          = "steer-L fwd"

                if reapproach_det:
                    reapproach_lost_frames = 0
                    cx, cy, area = reapproach_det
                    cx_frac = cx / w
                    log(f"RE-APPROACH {active_color} {steer_label} cx={cx_frac:.2f}")
                    wrong = (cx_frac > PILLAR_RED_WRONG_X) if active_color == "RED" \
                            else (cx_frac < PILLAR_GREEN_WRONG_X)
                    if wrong:
                        log(f"RE-APPROACH {active_color} still wrong side cx={cx_frac:.2f} -> PILLAR_WRONG_SIDE")
                        state            = "PILLAR_WRONG_SIDE"
                        wrong_side_start = now
                        danger_triggered = False
                        time.sleep(LOOP_DT); continue
                    if passed_check(cx_frac):
                        log("Pillar PASSED after recovery -> STRAIGHT")
                        passed_color   = active_color   # lock until it leaves frame
                        smoothed_steer = 0.0
                        state = "STRAIGHT"
                else:
                    reapproach_lost_frames += 1
                    log(f"RE-APPROACH {active_color} {steer_label} pillar lost {reapproach_lost_frames}")
                    if reapproach_lost_frames >= 5:
                        log(f"RE-APPROACH: {active_color} lost 5 frames -> STRAIGHT")
                        reapproach_lost_frames = 0
                        smoothed_steer = 0.0
                        state = "STRAIGHT"

            # ══════════════════════════════════════════════════════════════════
            #  APPROACHING  (from race.py, unchanged)
            # ══════════════════════════════════════════════════════════════════
            elif state == "APPROACHING":
                drive.set_steering(STEER_LEFT if turn_direction == "LEFT" else STEER_RIGHT)
                drive.forward(STRAIGHT_SPEED)
                if now >= approach_until:
                    Pw = precorner_wall_amount(frame)
                    if Pw >= PRECORNER_WALL_THRESHOLD:
                        log(f"wall in pre-corner box (P={Pw:.2f}) -- reversing before turn")
                        drive.stop()
                        state = "CORNER_BACKUP"; corner_backup_start = now
                    else:
                        log("turning (no stop)")
                        state = "TURNING_90"; turn_start = now

            # ══════════════════════════════════════════════════════════════════
            #  CORNER_BACKUP  (from race.py, unchanged)
            # ══════════════════════════════════════════════════════════════════
            elif state == "CORNER_BACKUP":
                drive.set_steering(STEER_CENTER)
                drive.backward(REVERSE_SPEED)
                Pw = precorner_wall_amount(frame)
                if Pw < PRECORNER_WALL_THRESHOLD or now - corner_backup_start >= BACKUP_MAX_S:
                    drive.stop()
                    log(f"pre-corner clear (P={Pw:.2f}) -- turning")
                    state = "TURNING_90"; turn_start = now

            # ══════════════════════════════════════════════════════════════════
            #  TURNING_90  (from race.py, unchanged)
            # ══════════════════════════════════════════════════════════════════
            elif state == "TURNING_90":
                Fw = front_wall_amount(frame)
                if Fw >= FRONT_WALL_THRESHOLD:
                    turn_remaining = max(0.2, TURN_90_DURATION_S - (now - turn_start))
                    log(f"FRONT WALL mid-corner (F={Fw:.2f}) -- backup, "
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
                        log("last corner done")
                    else:
                        state = "STRAIGHT"

            # ══════════════════════════════════════════════════════════════════
            #  REVERSING  (from race.py, handles both wall and corner backups)
            # ══════════════════════════════════════════════════════════════════
            elif state == "REVERSING":
                if resume_state == "TURNING_90":
                    drive.set_steering(STEER_RIGHT if turn_direction == "LEFT" else STEER_LEFT)
                    drive.backward(REVERSE_SPEED)
                    FwL, FwR = front_wall_halves(frame)
                    front_clear = (FwL + FwR) / 2.0 < FRONT_WALL_THRESHOLD
                    if front_clear or now - reverse_start >= BACKUP_MAX_S:
                        drive.stop()
                        line_blind_until = now + LINE_BLIND_AFTER_REVERSE_S
                        blind_until      = now + LINE_BLIND_AFTER_REVERSE_S
                        log(f"front clear -- resuming corner ({turn_remaining:.2f}s)")
                        turn_start = now - (TURN_90_DURATION_S - turn_remaining)
                        state = "TURNING_90"
                else:
                    if front_wall_side == "LEFT":
                        drive.set_steering(STEER_LEFT)
                    elif front_wall_side == "RIGHT":
                        drive.set_steering(STEER_RIGHT)
                    else:
                        drive.set_steering(STEER_CENTER)
                    drive.backward(REVERSE_SPEED)
                    if now - reverse_start >= backup_duration:
                        drive.stop()
                        line_blind_until = now + LINE_BLIND_AFTER_REVERSE_S
                        blind_until      = now + LINE_BLIND_AFTER_REVERSE_S
                        drive.set_steering(STEER_CENTER)
                        side_was        = front_wall_side
                        front_wall_side = None
                        stuck_ref       = None
                        smoothed_steer  = 0.0
                        if side_was == "LEFT":
                            recover_steer = "RIGHT"
                        elif side_was == "RIGHT":
                            recover_steer = "LEFT"
                        else:
                            recover_steer = None
                        if recover_steer:
                            log(f"backup done (wall {side_was}) -> RECOVER_FORWARD steer {recover_steer}")
                            state = "RECOVER_FORWARD"; recover_start = now
                        else:
                            log("backup done -> STRAIGHT")
                            state = "STRAIGHT"

            # ══════════════════════════════════════════════════════════════════
            #  RECOVER_FORWARD  (from race.py, unchanged)
            # ══════════════════════════════════════════════════════════════════
            elif state == "RECOVER_FORWARD":
                drive.set_steering(STEER_LEFT if recover_steer == "LEFT"
                                   else STEER_RIGHT if recover_steer == "RIGHT"
                                   else STEER_CENTER)
                drive.forward(RECOVER_SPEED)
                if now - recover_start >= RECOVER_FORWARD_DURATION_S:
                    drive.set_steering(STEER_CENTER)
                    state = "STRAIGHT"

            # ══════════════════════════════════════════════════════════════════
            #  FINISHING  (from race.py, unchanged)
            # ══════════════════════════════════════════════════════════════════
            elif state == "FINISHING":
                drive.set_steering(STEER_CENTER)
                drive.forward(STRAIGHT_SPEED)
                if now >= finish_at:
                    log("3 laps complete. Stopping.")
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


# ══════════════════════════════════════════════════════════════════════════════
#  PREVIEW / SNAPSHOT
# ══════════════════════════════════════════════════════════════════════════════

def preview(camera):
    print("preview (no motors) -- Ctrl+C to quit")
    try:
        while True:
            frame     = camera.get_frame()
            det_red   = detect_red(frame)
            det_green = detect_green(frame)
            lroi, rroi = wall_rois(frame)
            Lw, Rw    = wall_amount(lroi), wall_amount(rroi)
            camera.show_overlay(annotate(frame, det_red, det_green))

            if det_red:
                cx, cy, area = det_red
                cx_frac = cx / frame.shape[1]
                decision = ("PASSED" if cx_frac <= PILLAR_RED_PASSED_X
                            else "WRONG-SIDE" if cx_frac > PILLAR_RED_WRONG_X
                            else "STEER-RIGHT")
                print(f"RED cx={cx_frac:.2f} area={int(area)} -> {decision} "
                      f"Lw={Lw:.2f} Rw={Rw:.2f} line={line_color(frame)}")
            if det_green:
                cx, cy, area = det_green
                cx_frac = cx / frame.shape[1]
                decision = ("PASSED" if cx_frac >= PILLAR_GREEN_PASSED_X
                            else "WRONG-SIDE" if cx_frac < PILLAR_GREEN_WRONG_X
                            else "STEER-LEFT")
                print(f"GRN cx={cx_frac:.2f} area={int(area)} -> {decision} "
                      f"Lw={Lw:.2f} Rw={Rw:.2f} line={line_color(frame)}")
            if not det_red and not det_green:
                print(f"NO PILLAR Lw={Lw:.2f} Rw={Rw:.2f} line={line_color(frame)}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()


def snapshot(camera):
    frame     = camera.get_frame()
    det_red   = detect_red(frame)
    det_green = detect_green(frame)
    lroi, rroi = wall_rois(frame)
    print(f"Lw={wall_amount(lroi):.2f} Rw={wall_amount(rroi):.2f} "
          f"line={line_color(frame)} red={det_red} green={det_green}")
    cv2.imwrite("obstacle_snapshot.png", annotate(frame, det_red, det_green))
    print("saved obstacle_snapshot.png")
    camera.stop()


if __name__ == "__main__":
    mode   = sys.argv[1] if len(sys.argv) > 1 else "race"
    camera = Camera(); camera.start()
    if mode == "preview":
        preview(camera)
    elif mode == "snapshot":
        snapshot(camera)
    else:
        drive        = Drive()
        led          = LED(LED_PIN); led.off()
        start_button = Button(START_BUTTON_GPIO, pull_up=False, bounce_time=0.08)
        race(camera, drive, led, start_button)
