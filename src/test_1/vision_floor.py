# vision_floor.py

import cv2
import numpy as np


def build_floor_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_floor = np.array([0, 0, 80])
    upper_floor = np.array([180, 80, 255])

    floor_mask = cv2.inRange(hsv, lower_floor, upper_floor)

    kernel = np.ones((7, 7), np.uint8)

    floor_mask = cv2.morphologyEx(floor_mask, cv2.MORPH_CLOSE, kernel)
    floor_mask = cv2.morphologyEx(floor_mask, cv2.MORPH_OPEN, kernel)

    return floor_mask


def largest_white_segment(row):
    best_start = None
    best_end = None
    best_len = 0
    current_start = None

    for x, value in enumerate(row):
        if value > 0:
            if current_start is None:
                current_start = x
        else:
            if current_start is not None:
                current_end = x - 1
                current_len = current_end - current_start + 1

                if current_len > best_len:
                    best_start = current_start
                    best_end = current_end
                    best_len = current_len

                current_start = None

    if current_start is not None:
        current_end = len(row) - 1
        current_len = current_end - current_start + 1

        if current_len > best_len:
            best_start = current_start
            best_end = current_end
            best_len = current_len

    return best_start, best_end, best_len


def analyze_floor(frame):
    height, width = frame.shape[:2]
    floor_mask = build_floor_mask(frame)
    annotated = frame.copy()

    scanline_ys = [
        int(height * 0.58),
        int(height * 0.66),
        int(height * 0.74),
        int(height * 0.82),
    ]

    centers = []
    segment_lengths = []

    for y in scanline_ys:
        row = floor_mask[y, :]
        start_x, end_x, segment_len = largest_white_segment(row)

        cv2.line(annotated, (0, y), (width, y), (255, 255, 255), 1)

        if start_x is None or end_x is None or segment_len < 120:
            continue

        center_x = (start_x + end_x) // 2

        centers.append(center_x)
        segment_lengths.append(segment_len)

        cv2.circle(annotated, (start_x, y), 5, (255, 255, 255), -1)
        cv2.circle(annotated, (end_x, y), 5, (255, 255, 255), -1)
        cv2.circle(annotated, (center_x, y), 5, (255, 255, 255), -1)

    if centers:
        target_x = int(np.median(centers[-2:]))
        vision_ok = True
    else:
        target_x = width // 2
        vision_ok = False

    error = target_x - (width // 2)
    steering_value = int((error / (width // 2)) * 100)
    steering_value = max(-100, min(100, steering_value))

    # Front check: look at upper-middle area.
    # If the visible white floor ahead is small, a wall/corner is near.
    front_y1 = int(height * 0.38)
    front_y2 = int(height * 0.58)
    front_x1 = int(width * 0.25)
    front_x2 = int(width * 0.75)

    front_roi = floor_mask[front_y1:front_y2, front_x1:front_x2]
    white_pixels = cv2.countNonZero(front_roi)
    roi_area = front_roi.shape[0] * front_roi.shape[1]
    front_white_ratio = white_pixels / roi_area

    front_blocked = front_white_ratio < 0.35

    cv2.rectangle(
        annotated,
        (front_x1, front_y1),
        (front_x2, front_y2),
        (255, 255, 255),
        2,
    )

    target_y = int(height * 0.76)

    cv2.circle(annotated, (target_x, target_y), 10, (255, 255, 255), -1)
    cv2.line(annotated, (width // 2, height), (target_x, target_y), (255, 255, 255), 2)

    cv2.putText(
        annotated,
        f"err={error}, steer={steering_value}, front={front_white_ratio:.2f}, blocked={front_blocked}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )

    return {
        "floor_mask": floor_mask,
        "annotated": annotated,
        "target_x": target_x,
        "error": error,
        "steering_value": steering_value,
        "vision_ok": vision_ok,
        "centers": centers,
        "segment_lengths": segment_lengths,
        "front_white_ratio": front_white_ratio,
        "front_blocked": front_blocked,
    }