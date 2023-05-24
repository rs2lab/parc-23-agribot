import rospy
import cv2

import numpy as np
import vision as v
import constants as cons

from robot_perception import UcvSensorType
from cv_bridge import CvBridge


class UcvRobotPlanner:
    def __init__(self, control, perception):
        self._control = control
        self._perception = perception
        self._bridge = CvBridge()

    def plan(self):
        """Analyse the information from the perception mechanisms
        and determine the best course of action to be taken by the robot."""
        current_front_cam_state = self._perception.front_camera_state
        current_left_cam_state = self._perception.left_camera_state
        current_right_cam_state = self._perception.right_camera_state
        current_scanner_state = self._perception.laser_scan_state
        current_gps_state = self._perception.gps_state
        current_cmd_vel = self._perception.cmd_vel_state
        if current_front_cam_state is not None:
            frame = self._bridge.imgmsg_to_cv2(current_front_cam_state)
            image = v.remove_dark_area(frame)
            image = v.mask_image(image, cons.FRONT_MASK_01)
            s_image = v.detect_stake(image, image_is_hsv=False)
            p_image = v.detect_plants(image, image_is_hsv=False)
            s_lines = v.hough_lines(s_image, image_is_canny=False)
            p_lines = v.hough_lines(p_image, image_is_canny=False)
            image = v.draw_lines_on_image(frame, s_lines, (0, 10, 200))
            image = v.draw_lines_on_image(image, p_lines, (150, 10, 5))
            cv2.imshow('camera', image)
            cv2.waitKey(1)
        #TODO: proccess the information and return the best control strategy
        return None

