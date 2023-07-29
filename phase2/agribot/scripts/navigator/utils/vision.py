import cv2
import os
import numpy as np

from cv_bridge import CvBridge
from functools import reduce


FRONT_CAM_IMAGE_WIDTH = 672
FRONT_CAM_IMAGE_HEIGHT = 376

FRONT_MASK = cv2.imread('../images/front-mask02.png', 0)

BRIGHTNESS_ALPHA_ADJUST = 1.5
BRIGHTNESS_BETA_ADJUST = 10

FRONT_CAM_LEFT_POINT_REF = np.array((150, 360))
FRONT_CAM_RIGHT_POINT_REF = np.array((522, 360))

_BRIDGE_OBJ = CvBridge()


def imgmsg_to_cv2(imgmsg):
    return _BRIDGE_OBJ.imgmsg_to_cv2(imgmsg)


def cv2_to_imgmsg(img):
    return _BRIDGE_OBJ.cv2_to_imgmsg(img)


# The OpenCV HSV rate is a bit different from the most commonly
# used rates.
def cvt_common_hsv_to_opencv_hsv(hsv, ch01rate = 8):
    return np.array((ch01rate, 255, 255)) * hsv


FRONT_CAM_LANE_THRESHOLDING_LOWER_BOUND = cvt_common_hsv_to_opencv_hsv((0.055, 0.537, 0.241)) - 5
FRONT_CAM_LANE_THRESHOLDING_UPPER_BOUND = cvt_common_hsv_to_opencv_hsv((0.874, 1.0, 1.0)) + 5
FRONT_CAM_PLANTS_THRESHOLDING_LOWER_BOUND = cvt_common_hsv_to_opencv_hsv((0.153, 0.537, 0.0), 180) - 5
FRONT_CAM_PLANTS_THRESHOLDING_UPPER_BOUND = cvt_common_hsv_to_opencv_hsv((0.939, 1.0, 0.7), 180) + 5
LAT_CAM_PLANTS_THRESHOLDING_LOWER_BOUND = cvt_common_hsv_to_opencv_hsv((0.0, 0.3, 0.496), 180)
LAT_CAM_PLANTS_THRESHOLDING_UPPER_BOUND = cvt_common_hsv_to_opencv_hsv((0.931, 0.5, 1.0), 180)


def mask_image(image, mask):
    return cv2.bitwise_and(image, image, mask = mask)


def cvt_bgr_to_hsv(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


def adjust_image_brightness(image, alpha: float = BRIGHTNESS_ALPHA_ADJUST, beta: float = BRIGHTNESS_BETA_ADJUST):
    return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)


def canny(image, thresh1: int = 50, thresh2: int = 100, apply_blur: bool = False):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    if apply_blur is True:
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

    return cv2.Canny(gray, thresh1, thresh2)


def detect_color_profile(image, lower_bound, upper_bound, apply_blur: bool = False):
    if apply_blur is True:
        image = cv2.GaussianBlur(image, (5, 5), 0)
    color_mask = cv2.inRange(image, lower_bound, upper_bound)
    return cv2.bitwise_and(image, image, mask=color_mask)


def detect_front_cam_lanes(image, apply_hsv: bool = False):
    if apply_hsv is True:
        image = cvt_bgr_to_hsv(image=image)
    return detect_color_profile(image=image,
        lower_bound=FRONT_CAM_LANE_THRESHOLDING_LOWER_BOUND,
        upper_bound=FRONT_CAM_LANE_THRESHOLDING_UPPER_BOUND,
        apply_blur=False,
    )


def detect_front_cam_plants(image, apply_hsv: bool = False):
    if apply_hsv is True:
        image = cvt_bgr_to_hsv(image=image)
    return detect_color_profile(image=image,
        lower_bound=FRONT_CAM_PLANTS_THRESHOLDING_LOWER_BOUND,
        upper_bound=FRONT_CAM_PLANTS_THRESHOLDING_UPPER_BOUND,
        apply_blur=False,
    )


def hough_lines(image, min_line_len: int = 20, max_line_gap: int = 20, apply_canny: bool = True):
    if apply_canny is True:
        image = canny(image, apply_blur = True)
    return cv2.HoughLinesP(image, 2, np.pi / 180, 10, np.array([]), min_line_len, max_line_gap)


def draw_lines_on_image(image, lines, color_rgb = (12, 200, 90)):
    line_image = np.zeros_like(image)

    if lines is not None:
        for x1, y1, x2, y2 in lines.reshape(-1, 4):
            cv2.line(line_image, (x1, y1), (x2, y2), color_rgb, 10)

    return cv2.addWeighted(image, 0.8, line_image, 1, 1)


def make_line_detection(image, *, detect_fn, reduce_fn=None, crop_fn=None, mask=None):
    lines = None

    if image is not None:
        original_image = image
        if crop_fn is not None:
            image = crop_fn(image)

        if mask is not None:
            image = mask_image(image, mask=mask)

        image = detect_fn(image, apply_hsv=True)

        lines = hough_lines(image, apply_canny=True)

        # if image is not None and lines is not None:
        #     cv2.imshow('camera', draw_lines_on_image(original_image, lines))
        #     cv2.waitKey(1)

        if lines is not None and reduce_fn is not None:
            lines = reduce(reduce_fn, lines.reshape(-1, 4))
    return lines