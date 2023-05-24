import cv2

FRONT_MASK_01 = cv2.imread('../images/front_vision_mask_01.png', 0)
FRONT_MASK_02 = cv2.imread('../images/front_vision_mask_02.png', 0)

DEFAULT_SEG_CRT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

DARK_AREA_CROP_YY_THRESH = 325
LATERAL_CROP_XX_THRESH = 60
