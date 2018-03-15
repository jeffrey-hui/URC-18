import cv2
import numpy as np

min_c = np.array([30, 90, 70])
max_c = np.array([70, 240, 240])

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (12, 12))


def find_tennis_ball(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)
    thresh = cv2.inRange(hsv, min_c, max_c)
    erode = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    _, contours, _ = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_sorted = sorted(contours, key=lambda c: cv2.arcLength(c, True))
    if len(contours_sorted) >= 1:
        big_contour_area = cv2.contourArea(contours_sorted[-1])
        if big_contour_area > 120:
            convex_hull = cv2.convexHull(contours_sorted[-1])
            (x, y), radius = cv2.minEnclosingCircle(convex_hull)
            return x, y, radius
    return None
