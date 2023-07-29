import os
import cv2

import numpy as np

FRONT_CAM_IMAGE_WIDTH = 672
FRONT_CAM_IMAGE_HEIGHT = 376

FRONT_MASK = cv2.imread('../../../images/front-mask02.png', 0)

BRIGHTNESS_ALPHA_ADJUST = 1.5
BRIGHTNESS_BETA_ADJUST = 10


# The OpenCV HSV rate is a bit different from the most commonly
# used rates.
def cvt_common_hsv_to_opencv_hsv(hsv):
    return np.array((0.5, 255, 255)) * hsv


LANE_THRESHOLDING_LOWER_BOUND = cvt_common_hsv_to_opencv_hsv((0.055, 0.537, 0.241))
LANE_THRESHOLDING_UPPER_BOUND = cvt_common_hsv_to_opencv_hsv((0.874, 1.0, 1.0))


def mask_image(image, mask):
    return cv2.bitwise_and(image, image, mask = mask)


def cvt_rgb_to_hsv(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2HSV)


def adjust_image_brightness(image, alpha = BRIGHTNESS_ALPHA_ADJUST, beta = BRIGHTNESS_BETA_ADJUST):
    return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)


def canny(image, thresh1 = 50, thresh2 = 100, apply_blur = False):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    if apply_blur is True:
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

    return cv2.Canny(gray, thresh1, thresh2)


def detect_color_profile(image, lower_bound, upper_bound, apply_blur = False):
    if apply_blur is True:
        image = cv2.GaussianBlur(image, (5, 5), 0)
    color_mask = cv2.inRange(image, lower_bound, upper_bound)
    return cv2.bitwise_and(image, image, mask=color_mask)


def detect_lanes(image):
    return detect_color_profile(image=image,
        lower_bound=LANE_THRESHOLDING_LOWER_BOUND - 5,
        upper_bound=LANE_THRESHOLDING_UPPER_BOUND + 5,
        apply_blur=False,
    )


def hough_lines(image, min_line_len = 20, max_line_gap = 20, apply_canny = True):
    if apply_canny is True:
        image = canny(image, blur_image = True)
    return cv2.HoughLinesP(image, 2, np.pi / 180, 10, np.array([]), min_line_len, max_line_gap)


def draw_lines_on_image(image, lines, color_rgb = (12, 200, 90)):
    line_image = np.zeros_like(image)

    if lines is not None:
        for x1, y1, x2, y2 in lines.reshape(-1, 4):
            cv2.line(line_image, (x1, y1), (x2, y2), color_rgb, 10)

    return cv2.addWeighted(image, 0.8, line_image, 1, 1)
