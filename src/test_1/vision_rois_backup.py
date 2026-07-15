# vision_rois.py

import cv2
import numpy as np


def build_white_mask_lab(frame):
    """
    Detect white floor using LAB color space.
    LAB is less sensitive to lighting than raw RGB/BGR.
    """

    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    # LAB channels:
    # L = brightness
    # A = green-red
    # B = blue-yellow
    l_channel, a_channel, b_channel = cv2.split(lab)

    # White floor: high brightness, not too saturated.
    white_mask = cv2.inRange(
        lab,
        np.array([120, 105, 105]),
        np.array([255, 150, 155]),
    )

    kernel = np.ones((7, 7), np.uint8)

    white_mask = cv2.morphologyEx(
        white_mask,
        cv2.MORPH_CLOSE,
        kernel,
    )

    white_mask = cv2.morphologyEx(
        white_mask,
        cv2.MORPH_OPEN,
        kernel,
    )

    return white_mask


def count_white(mask, box):
    x1, y1, x2, y2 = box
    roi = mask[y1:y2, x1:x2]

    white_pixels = cv2.countNonZero(roi)
    area = roi.shape[0] * roi.shape[1]

    if area == 0:
        return 0, 0.0

    ratio = white_pixels / area

    return white_pixels, ratio


def analyze_rois(frame):
    height, width = frame.shape[:2]

    white_mask = build_white_mask_lab(frame)
    annotated = frame.copy()

    # These ROI positions are adapted to your low camera angle.
    # They are based on the visible road region in your screenshots.

    left_box = (
        int(width * 0.00),
        int(height * 0.45),
        int(width * 0.35),
        int(height * 0.65),
    )

    right_box = (
        int(width * 0.65),
        int(height * 0.45),
        int(width),
        int(height * 0.65),
    )

    front_box = (
        int(width * 0.20),
        int(height * 0.38),
        int(width * 0.80),
        int(height * 0.58),
    )

    left_pixels, left_ratio = count_white(white_mask, left_box)
    right_pixels, right_ratio = count_white(white_mask, right_box)
    front_pixels, front_ratio = count_white(white_mask, front_box)

    # Error convention:
    # positive error -> steer right
    # negative error -> steer left
    error = right_pixels - left_pixels

    # Normalize error to roughly [-100, 100]
    max_possible = max(left_pixels + right_pixels, 1)
    normalized_error = int((error / max_possible) * 100)

    # Front is blocked when little white floor is visible ahead.
    front_blocked = front_ratio < 0.25

    # Side opening idea:
    # If one side has much more white than the other, the track may be opening that way.
    side_difference = left_ratio - right_ratio

    left_open = side_difference > 0.25
    right_open = side_difference < -0.25

    # Draw ROIs
    cv2.rectangle(annotated, left_box[:2], left_box[2:], (255, 255, 255), 2)
    cv2.rectangle(annotated, right_box[:2], right_box[2:], (255, 255, 255), 2)
    cv2.rectangle(annotated, front_box[:2], front_box[2:], (255, 255, 255), 2)

    cv2.putText(
        annotated,
        f"L={left_ratio:.2f} R={right_ratio:.2f} F={front_ratio:.2f}",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )

    cv2.putText(
        annotated,
        f"err={normalized_error} front_blocked={front_blocked}",
        (20, 65),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )

    cv2.putText(
        annotated,
        f"left_open={left_open} right_open={right_open}",
        (20, 95),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )

    return {
        "white_mask": white_mask,
        "annotated": annotated,
        "left_pixels": left_pixels,
        "right_pixels": right_pixels,
        "front_pixels": front_pixels,
        "left_ratio": left_ratio,
        "right_ratio": right_ratio,
        "front_ratio": front_ratio,
        "error": normalized_error,
        "front_blocked": front_blocked,
        "left_open": left_open,
        "right_open": right_open,
    }
