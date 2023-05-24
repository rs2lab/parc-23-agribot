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
        #TODO: proccess the information and return the best control strategy

    def execute(self):
        """Execute the plan using the control mechanisms to achieve the goal."""
        front_camera_state = self._perception.front_camera_state
        if front_camera_state is not None:
            frame = self._bridge.imgmsg_to_cv2(front_camera_state)
            image = v.remove_dark_area(frame)
            image = v.mask_image(image, cons.FRONT_MASK_01)
            image = v.detect_plants(image, image_is_hsv=False)
            lines = v.hough_lines(image, image_is_canny=False)
            image = v.draw_lines_on_image(frame, lines, (0, 10, 200))
            cv2.imshow('camera', image)
            cv2.waitKey(1)
