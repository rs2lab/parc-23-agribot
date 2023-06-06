"""Local library containing computer vision algorithms used in this module."""

import cv2
import numpy as np

from cv_bridge import CvBridge

from constants import (
    DEFAULT_SEG_CRT,
    DARK_AREA_CROP_YY_THRESH,
    LATERAL_CROP_XX_THRESH,
)


_BRIDGE_OBJ = CvBridge()


def imgmsg_to_cv2(imgmsg):
    return _BRIDGE_OBJ.imgmsg_to_cv2(imgmsg)


def cv2_to_imgmsg(img):
    return _BRIDGE_OBJ.cv2_to_imgmsg(img)


def mask_image(image, mask):
    return cv2.bitwise_and(image, image, mask=mask)


def kmeans_segmentation(image, attempts = 10, k = 4, criteria = DEFAULT_SEG_CRT):
    td_img = np.float32(image.reshape((-1, 3)))
    _, label, center = cv2.kmeans(td_img, k, None, criteria, attempts, cv2.KMEANS_PP_CENTERS)

    center = np.uint8(center)
    res = center[label.flatten()]

    return res.reshape((image.shape))


def detect_color_profile(image, lower_bound, upper_bound, blur_image = False):
    if blur_image is True:
        image = cv2.GaussianBlur(image, (5, 5), 0)
    color_mask = cv2.inRange(image, lower_bound, upper_bound)
    return cv2.bitwise_and(image, image, mask=color_mask)


def detect_plants(image, lower_adjust = -10, upper_adjust = 10, image_is_hsv = False):
    if not image_is_hsv:
        image = convert_image_to_hsv(image)
    return detect_color_profile(
        image=image,
        lower_bound=np.array((63, 166, 137)) + lower_adjust,
        upper_bound=np.array([65, 208, 157]) + upper_adjust,
        blur_image=False,
    )


def detect_stake(image, lower_adjust = -10, upper_adjust = 10, image_is_hsv = False):
    if not image_is_hsv:
        image = convert_image_to_hsv(image)
    return detect_color_profile(
        image=image,
        lower_bound=np.array((20, 131, 166)) + lower_adjust,
        upper_bound=np.array((22, 192, 196)) + upper_adjust,
        blur_image=False,
    )


def detect_goal(image, lower_adjust = -10, upper_adjust = 10, image_is_hsv=False):
    if not image_is_hsv:
        image = convert_image_to_hsv(image)
    return detect_color_profile(
        image=image,
        lower_bound=np.array((0, 250, 230)) + (0, lower_adjust, lower_adjust),
        upper_bound=np.array((3, 255, 234)) + upper_adjust,
        blur_image=False,
    )


def convert_image_to_hsv(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2HSV_FULL)


def canny(image, thresh1 = 50, thresh2 = 100, blur_image = False):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    if blur_image is True:
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

    return cv2.Canny(gray, thresh1, thresh2)


def make_coordinates(image, line_params):
    slope, intercept = line_params
    y1 = image.shape[0]
    y2 = int(y1 * 0.8)
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])


def average_slope_intercep(image, lines):
    left_fit = []
    right_fit = []
    for x1, y1, x2, y2 in lines.reshape(-1, 4):
        params = np.polyfit((x1, x2), (y1, y2), 1)
        slope = params[0]
        intercept = params[1]
        if slope < 0:
            left_fit.append((slope, intercept))
        else:
            right_fit.append((slope, intercept))
    left_fit_average = np.average(left_fit, axis=0)
    right_fit_average = np.average(right_fit, axis=0)
    left_line = make_coordinates(image, left_fit_average)
    right_line = make_coordinates(image, right_fit_average)
    return np.array([left_line, right_line])


def hough_lines(image, min_line_len = 20, max_line_gap = 20, image_is_canny = False):
    if not image_is_canny:
        image = canny(image, blur_image = True)
    return cv2.HoughLinesP(image, 2, np.pi / 180, 10, np.array([]), min_line_len, max_line_gap)


def draw_lines_on_image(image, lines, color_bgr = (255, 0, 0)):
    line_image = np.zeros_like(image)

    if lines is not None:
        for x1, y1, x2, y2 in lines.reshape(-1, 4):
            cv2.line(line_image, (x1, y1), (x2, y2), color_bgr, 10)

    return cv2.addWeighted(image, 0.8, line_image, 1, 1)


def crop_front_image(image, yy_thresh = DARK_AREA_CROP_YY_THRESH):
    return image[:yy_thresh, ::]


def crop_lateral_image(image, xx_thresh = LATERAL_CROP_XX_THRESH):
    return image[::, xx_thresh:]
