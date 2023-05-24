import rospy
import cv2

import numpy as np
import vision as v

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
        pass # TODO

    def execute(self):
        """Execute the plan using the control mechanisms to achieve the goal."""
        front_camera_state = self._perception.front_camera_state
        if front_camera_state is not None:
            frame = self._bridge.imgmsg_to_cv2(front_camera_state)
            image = v.remove_dark_area(frame)
            image = v.detect_plants(image)
            lines = v.hough_lines(image)
            image = v.draw_lines_on_image(frame, lines, (0, 10, 200))
            cv2.imshow('camera', image)
            cv2.waitKey(1)
