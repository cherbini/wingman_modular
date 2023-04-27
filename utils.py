import numpy as np
from typing import Tuple
from typing import List
import cv2

def get_roi_area(roi):
    _, _, w, h = roi
    return w * h

def limit_lead_distance(yellow_x, yellow_y, green_x, green_y, max_distance=50):
    dx = green_x - yellow_x
    dy = green_y - yellow_y
    distance = np.sqrt(dx ** 2 + dy ** 2)

    if distance > max_distance:
        green_x = yellow_x + (dx / distance) * max_distance
        green_y = yellow_y + (dy / distance) * max_distance

    return green_x, green_y

def get_interception_point(x: float, y: float, velocity: float, direction: float, lead_time: float) -> Tuple[float, float]:
    direction_rad = np.deg2rad(direction)
    delta_x = velocity * np.cos(direction_rad) * lead_time
    delta_y = velocity * np.sin(direction_rad) * lead_time
    interception_x = x + delta_x
    interception_y = y + delta_y
    return interception_x, interception_y

def extract_regions_of_interest(diff, threshold=0.2, min_side_length=25):
    gray_diff = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    ret, mask = cv2.threshold(gray_diff, threshold * 255, 255, cv2.THRESH_BINARY)
    mask = mask.astype(np.uint8)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largest_roi = None
    largest_area = 0

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)

        if w >= min_side_length and h >= min_side_length:
            area = w * h
            if area > largest_area:
                largest_area = area
                largest_roi = (x, y, w, h)

    return [largest_roi] if largest_roi else []

def get_center(x: float, y: float, w: float, h: float) -> List[float]:
    return [x + w / 2, y + h / 2, 1]

def process_motion_data(prev_roi, curr_roi, frame_width, frame_height):
    prev_x, prev_y, prev_w, prev_h = prev_roi
    curr_x, curr_y, curr_w, curr_h = curr_roi

    x_center = frame_width / 2
    y_center = frame_height / 2

    # Calculate azimuth
    x_diff = (curr_x + curr_w / 2) - x_center
    y_diff = (curr_y + curr_h / 2) - y_center
    azimuth = np.arctan2(y_diff, x_diff) * 180 / np.pi

    # Calculate velocity and direction
    delta_x = curr_x - prev_x
    delta_y = curr_y - prev_y
    velocity = np.sqrt(delta_x ** 2 + delta_y ** 2)
    direction = np.arctan2(delta_y, delta_x) * 180 / np.pi

    return azimuth, velocity, direction
