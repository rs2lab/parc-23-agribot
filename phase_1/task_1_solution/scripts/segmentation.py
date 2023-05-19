import numpy as np
import cv2


DEFAULT_SEG_CRT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
MASK = cv2.imread('../images/robot_camera_mask.png', 0)


def mask_image(image, mask):
    return cv2.bitwise_and(image, image, mask=mask)


def segment_image(image, attempts = 10, k = 4, criteria = DEFAULT_SEG_CRT):
    td_img = np.float32(image.reshape((-1, 3)))
    _, label, center = cv2.kmeans(td_img, k, None, criteria, attempts, cv2.KMEANS_PP_CENTERS)

    center = np.uint8(center)
    res = center[label.flatten()]

    return res.reshape((image.shape))


def detect_segmented_color_profile(image, lower_bound, upper_bound):
    image_masked = mask_image(cv2.medianBlur(image, 3), MASK)
    segmented_image = segment_image(image_masked)
    color_mask = cv2.inRange(segmented_image, lower_bound, upper_bound)
    return cv2.bitwise_and(segmented_image, segmented_image, mask=color_mask)


def detect_plants(image, lower_adjust = -10, upper_adjust = 5):
    return detect_segmented_color_profile(
        image=image,
        lower_bound=np.array([90, 120, 40]) + lower_adjust,
        upper_bound=np.array([100, 220, 70]) + upper_adjust,
    )


def detect_ground(image, lower_adjust = -4, upper_adjust = 10):
    return detect_segmented_color_profile(
        image=image,
        lower_bound=np.array([105, 90, 55]) + lower_adjust,
        upper_bound=np.array([165, 140, 120]) + upper_adjust,
    )