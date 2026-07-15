import cv2
import numpy as np


WALL_VISIBLE_THRESHOLD = 0.15
WALL_MISSING_THRESHOLD = 0.03
WALL_BALANCE_EXIT_THRESHOLD = 0.12


def build_white_mask_lab(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    white_mask = cv2.inRange(
        lab,
        np.array([120, 105, 105]),
        np.array([255, 150, 155]),
    )

    kernel = np.ones((7, 7), np.uint8)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)

    return white_mask


def build_black_mask_lab(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    black_mask = cv2.inRange(
        lab,
        np.array([0, 0, 0]),
        np.array([80, 255, 255]),
    )

    kernel = np.ones((7, 7), np.uint8)
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)

    return black_mask


def count_pixels(mask, box):
    x1, y1, x2, y2 = box
    roi = mask[y1:y2, x1:x2]

    pixels = cv2.countNonZero(roi)
    area = roi.shape[0] * roi.shape[1]

    if area == 0:
        return 0, 0.0

    return pixels, pixels / area


def analyze_rois(frame):
    height, width = frame.shape[:2]

    white_mask = build_white_mask_lab(frame)
    black_mask = build_black_mask_lab(frame)
    annotated = frame.copy()

    left_white_box = (
        int(width * 0.00),
        int(height * 0.60),
        int(width * 0.35),
        int(height * 0.80),
    )

    right_white_box = (
        int(width * 0.65),
        int(height * 0.60),
        int(width * 1.00),
        int(height * 0.80),
    )

    left_wall_box = (
        int(width * 0.00),
        int(height * 0.30),
        int(width * 0.42),
        int(height * 0.78),
    )

    right_wall_box = (
        int(width * 0.58),
        int(height * 0.30),
        int(width * 1.00),
        int(height * 0.78),
    )

    left_white_pixels, left_white_ratio = count_pixels(white_mask, left_white_box)
    right_white_pixels, right_white_ratio = count_pixels(white_mask, right_white_box)

    left_wall_pixels, left_wall_ratio = count_pixels(black_mask, left_wall_box)
    right_wall_pixels, right_wall_ratio = count_pixels(black_mask, right_wall_box)

    error = right_white_pixels - left_white_pixels
    total_white = max(left_white_pixels + right_white_pixels, 1)
    normalized_error = int((error / total_white) * 100)

    left_wall_visible = left_wall_ratio > WALL_VISIBLE_THRESHOLD
    right_wall_visible = right_wall_ratio > WALL_VISIBLE_THRESHOLD

    left_wall_missing = left_wall_ratio < WALL_MISSING_THRESHOLD
    right_wall_missing = right_wall_ratio < WALL_MISSING_THRESHOLD

    only_left_wall_visible = left_wall_visible and right_wall_missing
    only_right_wall_visible = right_wall_visible and left_wall_missing

    wall_difference = abs(left_wall_ratio - right_wall_ratio)
    walls_balanced = wall_difference < WALL_BALANCE_EXIT_THRESHOLD

    cv2.rectangle(annotated, left_white_box[:2], left_white_box[2:], (255, 255, 255), 2)
    cv2.rectangle(annotated, right_white_box[:2], right_white_box[2:], (255, 255, 255), 2)

    cv2.rectangle(annotated, left_wall_box[:2], left_wall_box[2:], (180, 180, 180), 2)
    cv2.rectangle(annotated, right_wall_box[:2], right_wall_box[2:], (180, 180, 180), 2)

    cv2.putText(
        annotated,
        f"Lwhite={left_white_ratio:.2f} Rwhite={right_white_ratio:.2f} err={normalized_error}",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )

    cv2.putText(
        annotated,
        f"Lwall={left_wall_ratio:.2f} Rwall={right_wall_ratio:.2f}",
        (20, 65),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )

    cv2.putText(
        annotated,
        f"onlyL={only_left_wall_visible} onlyR={only_right_wall_visible} balanced={walls_balanced}",
        (20, 95),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )

    return {
        "white_mask": white_mask,
        "black_mask": black_mask,
        "annotated": annotated,

        "left_white_ratio": left_white_ratio,
        "right_white_ratio": right_white_ratio,
        "left_white_pixels": left_white_pixels,
        "right_white_pixels": right_white_pixels,

        "left_wall_ratio": left_wall_ratio,
        "right_wall_ratio": right_wall_ratio,
        "left_wall_pixels": left_wall_pixels,
        "right_wall_pixels": right_wall_pixels,

        "left_wall_visible": left_wall_visible,
        "right_wall_visible": right_wall_visible,
        "left_wall_missing": left_wall_missing,
        "right_wall_missing": right_wall_missing,

        "only_left_wall_visible": only_left_wall_visible,
        "only_right_wall_visible": only_right_wall_visible,

        "wall_difference": wall_difference,
        "walls_balanced": walls_balanced,

        "error": normalized_error,
    }
