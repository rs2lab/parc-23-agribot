import cv2
import numpy as np

FRONT_MASK_01 = cv2.imread('../images/front_vision_mask_01.png', 0)
FRONT_MASK_02 = cv2.imread('../images/front_vision_mask_02.png', 0)

DEFAULT_SEG_CRT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

DARK_AREA_CROP_YY_THRESH = 325

#NOTE: remember to remove this also from the lateral points
# xx when searching for the closest line.
LATERAL_CROP_XX_THRESH = 60

LATERAL_LEFT_VISION_POINT = np.array((320, 300))
LATERAL_RIGHT_VISION_POINT = np.array((320, 180))

CROPPED_LATERAL_LEFT_VISION_POINT = LATERAL_LEFT_VISION_POINT - (LATERAL_CROP_XX_THRESH, 0)
CROPPED_LATERAL_RIGHT_VISION_POINT = LATERAL_RIGHT_VISION_POINT - (LATERAL_CROP_XX_THRESH, 0)

FRONT_VISION_LEFT_POINT = np.array((200, 300))
FRONT_VISION_RIGHT_POINT = np.array((440, 300))
