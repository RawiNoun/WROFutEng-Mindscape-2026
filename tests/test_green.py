#!/usr/bin/env python3
"""
test_green.py  --  GREEN pillar behavior test
Robot must pass the GREEN pillar on the LEFT side.

Start: 2 second countdown (no button press needed).

Wrong-side recovery:
  1. Steer OPPOSITE (right) + drive BACKWARD for 0.8s
  2. Steer CORRECT (left) + drive FORWARD
  3. Keep driving forward (steering left) until the pillar is actually PASSED
     (not a fixed timer -- it re-checks cx_frac every frame)

Includes:
  - Front wall backup (left hit, right hit, both)
  - Pre-corner danger box backup
  - Stuck detection + reverse
  - Wall centering when no pillar visible

Run:
    python3 test_green.py          # full motor test (2s countdown then starts)
    python3 test_green.py preview  # vision only, no motors
"""

import sys
import time
from time import monotonic
import cv2
import numpy as np
from gpiozero import LED
from camera import Camera
from drive import Drive

# ── color ranges ──────────────────────────────────────────────────────────────
LOWER_GREEN = np.array([ 40,  70,  50])
UPPER_GREEN = np.array([ 85, 255, 255])

# ── driving ───────────────────────────────────────────────────────────────────
STRAIGHT_SPEED  = 0.30
REVERSE_SPEED   = 0.30
STEER_CENTER    = 0
STEER_LEFT      = -70
STEER_RIGHT     =  70

# Separate smoothing: pillar steering snaps fast, wall centering stays gentle
PILLAR_SMOOTH_ALPHA = 0.15   # lower = snappier / stronger pillar correction
WALL_SMOOTH_ALPHA   = 0.35   # matches race.py wall centering feel

# ── wall centering ────────────────────────────────────────────────────────────
WALL_ROI_TOP       = 0.15
WALL_ROI_BOTTOM    = 0.40
WALL_ROI_WIDTH     = 0.20
WALL_L_MAX         = 110
CAM_STEER_DEADBAND = 0.06
CAM_STEER_GAIN     = 90.0
CAM_MAX_OFFSET     = 70

# ── front wall ────────────────────────────────────────────────────────────────
FRONT_ROI_TOP        = 0.30
FRONT_ROI_BOTTOM     = 0.50
FRONT_ROI_LEFT       = 0.25
FRONT_ROI_RIGHT      = 0.75
FRONT_WALL_THRESHOLD = 0.30
BACKUP_ONE_SIDE_S    = 0.4
BACKUP_BOTH_S        = 0.7
LINE_BLIND_AFTER_REVERSE_S = 1.0

# ── pre-corner danger box ─────────────────────────────────────────────────────
PRECORNER_Y_TOP_PX       = 80
PRECORNER_Y_BOTTOM_PX    = 120
PRECORNER_X_LEFT         = FRONT_ROI_LEFT  + 0.15
PRECORNER_X_RIGHT        = FRONT_ROI_RIGHT - 0.15
PRECORNER_WALL_THRESHOLD = 0.30
BACKUP_MAX_S             = 0.4

# ── stuck detection ───────────────────────────────────────────────────────────
STUCK_FRAME_DIFF_THRESHOLD = 5.0
STUCK_TIME_S               = 1.5
REVERSE_DURATION_S         = 1.0

# ── pillar ────────────────────────────────────────────────────────────────────
PILLAR_ROI_TOP    = 0.10   # detect earlier / farther away
PILLAR_ROI_BOTTOM = 0.75
PILLAR_ROI_LEFT   = 0.05
PILLAR_ROI_RIGHT  = 0.95
PILLAR_MIN_AREA   = 700

# GREEN passed threshold: cx_frac >= this means robot is LEFT of pillar ✓
PILLAR_GREEN_PASSED_X = 0.62

# GREEN wrong-side threshold: cx_frac < this means robot is on the WRONG (right) side
PILLAR_GREEN_WRONG_X  = 0.6

# Wrong-side recovery tuning
WRONG_SIDE_REVERSE_S        = 1   # steer opposite + reverse for this long
DANGER_WRONG_SIDE_REVERSE_S = WRONG_SIDE_REVERSE_S / 2   # halved when triggered from danger box

LED_PIN     = 22
LOOP_DT     = 0.06
COUNTDOWN_S = 2


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


def pillar_in_danger_box(detection, frame_shape):
    """True if the detected pillar centroid falls inside the danger box region."""
    if detection is None:
        return False
    h, w = frame_shape[:2]
    y1 = max(0, PRECORNER_Y_TOP_PX)
    y2 = min(h, PRECORNER_Y_BOTTOM_PX)
    x1 = int(PRECORNER_X_LEFT  * w)
    x2 = int(PRECORNER_X_RIGHT * w)
    cx, cy, area = detection
    return (x1 <= cx <= x2) and (y1 <= cy <= y2)


def frame_diff(a, b):
    ga = cv2.resize(cv2.cvtColor(a, cv2.COLOR_BGR2GRAY), (80, 60))
    gb = cv2.resize(cv2.cvtColor(b, cv2.COLOR_BGR2GRAY), (80, 60))
    return cv2.absdiff(ga, gb).mean()


def detect_green(frame):
    """Detect the largest GREEN pillar. Returns (cx, cy, area) or None."""
    h, w = frame.shape[:2]
    y1 = int(PILLAR_ROI_TOP    * h)
    y2 = int(PILLAR_ROI_BOTTOM * h)
    x1 = int(PILLAR_ROI_LEFT   * w)
    x2 = int(PILLAR_ROI_RIGHT  * w)
    roi = frame[y1:y2, x1:x2]
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


def annotate(frame, detection, status, Lw, Rw, FwL, FwR, Pw):
    out = frame.copy()
    h, w = out.shape[:2]

    # Wall ROI boxes (green)
    t, b = int(WALL_ROI_TOP * h), int(WALL_ROI_BOTTOM * h)
    ww   = int(WALL_ROI_WIDTH * w)
    cv2.rectangle(out, (0, t),      (ww, b), (0, 255, 0), 2)
    cv2.rectangle(out, (w - ww, t), (w,  b), (0, 255, 0), 2)
    cv2.putText(out, f"L={Lw:.2f}", (4, t + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(out, f"R={Rw:.2f}", (w - ww + 4, t + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Front wall box - split into LEFT and RIGHT halves
    ft, fb = int(FRONT_ROI_TOP * h), int(FRONT_ROI_BOTTOM * h)
    fl, fr = int(FRONT_ROI_LEFT * w), int(FRONT_ROI_RIGHT * w)
    fmid   = (fl + fr) // 2
    fwL_col = (0, 0, 255) if FwL >= FRONT_WALL_THRESHOLD else (255, 255, 0)
    fwR_col = (0, 0, 255) if FwR >= FRONT_WALL_THRESHOLD else (255, 255, 0)
    cv2.rectangle(out, (fl, ft),   (fmid, fb), fwL_col, 2)
    cv2.rectangle(out, (fmid, ft), (fr, fb),   fwR_col, 2)
    cv2.putText(out, f"FL={FwL:.2f}", (fl + 4, ft + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, fwL_col, 2)
    cv2.putText(out, f"FR={FwR:.2f}", (fmid + 4, ft + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, fwR_col, 2)

    # Pillar ROI boundary (white)
    pr_t = int(PILLAR_ROI_TOP    * h)
    pr_b = int(PILLAR_ROI_BOTTOM * h)
    pr_l = int(PILLAR_ROI_LEFT   * w)
    pr_r = int(PILLAR_ROI_RIGHT  * w)
    cv2.rectangle(out, (pr_l, pr_t), (pr_r, pr_b), (200, 200, 200), 1)

    # GREEN passed threshold line (blue vertical) -- right side of frame
    px = int(PILLAR_GREEN_PASSED_X * w)
    cv2.line(out, (px, pr_t), (px, pr_b), (0, 200, 0), 2)
    cv2.putText(out, "PASS>", (px + 4, pr_t + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2)

    # GREEN wrong-side threshold line -- left side of frame
    wx = int(PILLAR_GREEN_WRONG_X * w)
    cv2.line(out, (wx, pr_t), (wx, pr_b), (0, 128, 0), 2)
    cv2.putText(out, "<WRONG", (wx - 70, pr_t + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 0), 2)

    # Pillar dot + label
    if detection:
        cx, cy, area = detection
        cx_frac = cx / w
        if cx_frac >= PILLAR_GREEN_PASSED_X:
            label, color = "PASSED", (0, 255, 100)
        elif cx_frac < PILLAR_GREEN_WRONG_X:
            label, color = "WRONG-SIDE", (0, 180, 0)
        else:
            label, color = "STEER-L", (0, 255, 0)
        cv2.circle(out, (cx, cy), 10, (0, 200, 0), -1)
        cv2.circle(out, (cx, cy), 12, (255, 255, 255), 2)
        cv2.line(out, (cx, pr_t), (cx, pr_b), (0, 180, 0), 1)
        cv2.putText(out, f"GRN {label} a={int(area)} cx={cx_frac:.2f}",
                    (cx + 14, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    cv2.putText(out, status, (5, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 0), 2)
    return out


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN RUN LOOP
# ══════════════════════════════════════════════════════════════════════════════

def run(camera, drive, led):
    drive.set_steering(STEER_CENTER)

    # 2 second countdown instead of button press
    for i in range(COUNTDOWN_S, 0, -1):
        frame      = camera.get_frame()
        detection  = detect_green(frame)
        lroi, rroi = wall_rois(frame)
        Lw, Rw     = wall_amount(lroi), wall_amount(rroi)
        FwL, FwR   = front_wall_halves(frame)
        Pw         = precorner_wall_amount(frame)
        print(f"Starting in {i}...")
        camera.show_overlay(annotate(frame, detection, f"STARTING IN {i}",
                                     Lw, Rw, FwL, FwR, Pw))
        time.sleep(1.0)
    print("GO")

    smoothed_steer   = 0.0
    state            = "STRAIGHT"
    reverse_start    = 0.0
    backup_duration  = REVERSE_DURATION_S
    front_wall_side  = None
    blind_until      = 0.0
    stuck_ref        = None
    stuck_since      = monotonic()
    wrong_side_start      = 0.0
    danger_triggered      = False
    reapproach_lost_frames = 0
    status                = "STARTING"

    try:
        while True:
            now        = monotonic()
            frame      = camera.get_frame()
            h, w       = frame.shape[:2]
            lroi, rroi = wall_rois(frame)
            Lw, Rw     = wall_amount(lroi), wall_amount(rroi)
            FwL, FwR   = front_wall_halves(frame)
            Pw         = precorner_wall_amount(frame)
            detection  = detect_green(frame)

            # ── STRAIGHT ──────────────────────────────────────────────────────
            if state == "STRAIGHT":

                # 1. Pre-corner danger box check (skip re-trigger during blind cooldown)
                if Pw >= PRECORNER_WALL_THRESHOLD and now >= blind_until:
                    if pillar_in_danger_box(detection, frame.shape):
                        # GREEN pillar in the danger box -> wrong-side style recovery, halved
                        print(f"DANGER BOX GREEN detected P={Pw:.2f} -> wrong-side recovery (halved)")
                        state            = "PILLAR_WRONG_SIDE"
                        wrong_side_start = now
                        danger_triggered = True
                        time.sleep(LOOP_DT); continue
                    else:
                        # Plain wall -> normal backup
                        print(f"DANGER BOX WALL P={Pw:.2f} -> reversing")
                        drive.stop()
                        state           = "REVERSING"
                        reverse_start   = now
                        backup_duration = BACKUP_BOTH_S
                        front_wall_side = None
                        time.sleep(LOOP_DT); continue

                # 2. Front wall check (skip re-trigger during blind cooldown after a backup)
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
                    print(f"FRONT WALL {front_wall_side} FL={FwL:.2f} FR={FwR:.2f}")
                    state = "REVERSING"; reverse_start = now; stuck_ref = None
                    time.sleep(LOOP_DT); continue

                # 3. Stuck detection
                if stuck_ref is None:
                    stuck_ref = frame.copy(); stuck_since = now
                else:
                    if frame_diff(frame, stuck_ref) >= STUCK_FRAME_DIFF_THRESHOLD:
                        stuck_ref = frame.copy(); stuck_since = now
                    elif now - stuck_since >= STUCK_TIME_S:
                        print("STUCK -> reversing")
                        front_wall_side = None
                        backup_duration = REVERSE_DURATION_S
                        state = "REVERSING"; reverse_start = now; stuck_ref = None
                        time.sleep(LOOP_DT); continue

                # 4. Pillar logic
                if detection and now >= blind_until:
                    cx, cy, area = detection
                    cx_frac = cx / w

                    if cx_frac < PILLAR_GREEN_WRONG_X:
                        # WRONG SIDE -> go recover
                        print(f"WRONG SIDE detected cx={cx_frac:.2f} -> recovering")
                        state            = "PILLAR_WRONG_SIDE"
                        wrong_side_start = now
                        danger_triggered = False
                        time.sleep(LOOP_DT); continue

                    passed = cx_frac >= PILLAR_GREEN_PASSED_X
                    if not passed:
                        target = float(STEER_LEFT)   # steer LEFT to pass on the left
                        alpha  = PILLAR_SMOOTH_ALPHA
                        status = f"GRN STEER-L cx={cx_frac:.2f}"
                    else:
                        # Pillar still visible but passed -- go straight, do NOT
                        # hand to wall centering yet (pillar contaminates wall ROIs)
                        target = STEER_CENTER
                        alpha  = PILLAR_SMOOTH_ALPHA
                        status = f"GRN PASSED (still visible) -> straight cx={cx_frac:.2f}"
                else:
                    error  = Lw - Rw
                    target = clamp(CAM_STEER_GAIN * error,
                                   -CAM_MAX_OFFSET, CAM_MAX_OFFSET) \
                             if abs(error) > CAM_STEER_DEADBAND else STEER_CENTER
                    alpha  = WALL_SMOOTH_ALPHA
                    status = f"NO GRN -> wall L={Lw:.2f} R={Rw:.2f}"

                smoothed_steer = ((1 - alpha) * smoothed_steer + alpha * target)
                drive.set_steering(smoothed_steer)
                drive.forward(STRAIGHT_SPEED)
                print(status + f"  steer={smoothed_steer:+.0f}")

            # ── PILLAR_WRONG_SIDE: steer OPPOSITE (right) + reverse ────────────
            elif state == "PILLAR_WRONG_SIDE":
                duration = DANGER_WRONG_SIDE_REVERSE_S if danger_triggered else WRONG_SIDE_REVERSE_S
                status = f"WRONG-SIDE: reverse+steer-R  t={now - wrong_side_start:.2f}s" \
                         f"{' (danger, halved)' if danger_triggered else ''}"
                drive.set_steering(STEER_RIGHT)   # opposite of correct (left) side
                drive.backward(REVERSE_SPEED)
                print(status)

                if now - wrong_side_start >= duration:
                    drive.stop()
                    smoothed_steer         = 0.0
                    reapproach_lost_frames = 0
                    print("WRONG-SIDE reverse done -> steering correct + forward")
                    state = "PILLAR_REAPPROACH"

            # ── PILLAR_REAPPROACH: steer LEFT + forward until pillar reaches PASSED ─
            elif state == "PILLAR_REAPPROACH":
                drive.set_steering(STEER_LEFT)    # swing nose left forward so pillar drifts toward PASSED_X
                smoothed_steer = float(STEER_LEFT)
                drive.forward(STRAIGHT_SPEED)

                if detection:
                    reapproach_lost_frames = 0   # reset grace counter while pillar visible
                    cx, cy, area = detection
                    cx_frac = cx / w
                    status = f"RE-APPROACH steer-L fwd cx={cx_frac:.2f}"
                    if cx_frac >= PILLAR_GREEN_PASSED_X:
                        print("Pillar PASSED after recovery -> STRAIGHT")
                        smoothed_steer = 0.0
                        state = "STRAIGHT"
                else:
                    reapproach_lost_frames += 1
                    status = f"RE-APPROACH steer-L fwd (pillar lost {reapproach_lost_frames})"
                    # Only give up after 5 consecutive frames with no detection
                    if reapproach_lost_frames >= 5:
                        print("RE-APPROACH: pillar lost for 5 frames -> STRAIGHT")
                        reapproach_lost_frames = 0
                        smoothed_steer = 0.0
                        state = "STRAIGHT"
                print(status)

            # ── REVERSING (front wall / stuck) ─────────────────────────────────
            elif state == "REVERSING":
                status = f"REVERSING ({front_wall_side})"
                if front_wall_side == "LEFT":
                    drive.set_steering(STEER_RIGHT)   # left side hit -> steer away = right
                elif front_wall_side == "RIGHT":
                    drive.set_steering(STEER_LEFT)    # right side hit -> steer away = left
                else:
                    drive.set_steering(STEER_CENTER)
                drive.backward(REVERSE_SPEED)

                if now - reverse_start >= backup_duration:
                    drive.stop()
                    drive.set_steering(STEER_CENTER)
                    blind_until     = now + LINE_BLIND_AFTER_REVERSE_S
                    front_wall_side = None
                    smoothed_steer  = 0.0
                    stuck_ref       = None
                    print("Backup done -> STRAIGHT")
                    state = "STRAIGHT"

            camera.show_overlay(annotate(frame, detection, status, Lw, Rw, FwL, FwR, Pw))
            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        pass
    finally:
        drive.stop()
        drive.set_steering(STEER_CENTER)
        drive.cleanup()
        camera.stop()


# ══════════════════════════════════════════════════════════════════════════════
#  PREVIEW
# ══════════════════════════════════════════════════════════════════════════════

def preview(camera):
    print("GREEN preview -- Ctrl+C to quit")
    try:
        while True:
            frame      = camera.get_frame()
            h, w       = frame.shape[:2]
            detection  = detect_green(frame)
            lroi, rroi = wall_rois(frame)
            Lw, Rw     = wall_amount(lroi), wall_amount(rroi)
            FwL, FwR   = front_wall_halves(frame)
            Pw         = precorner_wall_amount(frame)
            if detection:
                cx, cy, area = detection
                cx_frac  = cx / w
                if cx_frac >= PILLAR_GREEN_PASSED_X:
                    decision = "PASSED"
                elif cx_frac < PILLAR_GREEN_WRONG_X:
                    decision = "WRONG-SIDE"
                else:
                    decision = "STEER-LEFT"
                print(f"GRN cx={cx_frac:.2f} area={int(area)} -> {decision} "
                      f"| FL={FwL:.2f} FR={FwR:.2f} P={Pw:.2f}")
            else:
                print(f"NO GRN L={Lw:.2f} R={Rw:.2f} "
                      f"FL={FwL:.2f} FR={FwR:.2f} P={Pw:.2f}")
            camera.show_overlay(annotate(frame, detection, "PREVIEW",
                                         Lw, Rw, FwL, FwR, Pw))
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()


if __name__ == "__main__":
    mode   = sys.argv[1] if len(sys.argv) > 1 else "run"
    camera = Camera(); camera.start()
    if mode == "preview":
        preview(camera)
    else:
        drive = Drive()
        led   = LED(LED_PIN); led.off()
        run(camera, drive, led)
