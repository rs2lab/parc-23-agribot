import cv2
import numpy as np

FRONT_MASK_01 = cv2.imread('../images/front_vision_mask_01.png', 0)
FRONT_MASK_02 = cv2.imread('../images/front_vision_mask_02.png', 0)

DEFAULT_SEG_CRT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

DARK_AREA_CROP_YY_THRESH = 325
LATERAL_CROP_XX_THRESH = 60

RIGHT_VISION_HIGH_POINT=np.array((355, 145))
RIGHT_VISION_LOW_POINT=np.array((165, 145))

LEFT_VISION_HIGH_POINT=np.array((415, 315))
LEFT_VISION_LOW_POINT=np.array((215, 325))

FRONT_VISION_LEFT_POINT=np.array((200, 300))
FRONT_VISION_RIGHT_POINT=np.array((440, 300))
