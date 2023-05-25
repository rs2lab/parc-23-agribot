import rospy
import cv2

import numpy as np
import vision as v
import constants as cons
import ruler as r

from geometry_msgs.msg import Twist
from robot_perception import UcvSensorType
from cv_bridge import CvBridge
from functools import reduce


class UcvSimpleActionPlan:
    def __init__(self, x, theta, secs=0):
        self.secs = secs
        self.theta = theta
        self.x = x

    def to_twist(self):
        twist = Twist()
        twist.linear.x = self.x
        twist.angular.z = self.theta
        return twist

    @classmethod
    def from_twist(cls, twist, secs=0):
        return UcvSimpleActionPlan(
            theta=twist.angular.z,
            x=twist.linear.x,
            secs=secs,
        )


class UcvRobotPlanner:
    def __init__(self, control, perception, default_plan_timeslot_in_secs=1):
        self._control = control
        self._perception = perception
        self._bridge = CvBridge()

        self._default_plan_secs = default_plan_timeslot_in_secs

        self._left_cam_line_reducer = r.define_lateral_line_reducer(
            highest_point=cons.LEFT_VISION_HIGH_POINT,
            lowest_point=cons.LEFT_VISION_LOW_POINT
        )

        self._right_cam_line_reducer = r.define_lateral_line_reducer(
            highest_point=cons.RIGHT_VISION_HIGH_POINT,
            lowest_point=cons.RIGHT_VISION_LOW_POINT
        )

        self._front_left_line_reducer = r.define_frontal_line_reducer(
            point=cons.FRONT_VISION_LEFT_POINT
        )

        self._front_right_line_reducer = r.define_frontal_line_reducer(
            point=cons.FRONT_VISION_RIGHT_POINT
        )

    def _calculate_detected_line(self, cam_state, *, crop_fn=None, detection_fn, reduce_fn):
        line = None
        if cam_state is not None:
            image = self._bridge.imgmsg_to_cv2(cam_state)

            if crop_fn is not None:
                image = crop_fn(image)

            image = detection_fn(image, image_is_hsv=False)
            lines = v.hough_lines(image, image_is_canny=False)

            if lines is not None:
                line = reduce(reduce_fn, lines.reshape(-1, 4))
        return line

    def plan(self, secs=None):
        """Analyse the information from the perception mechanisms
        and determine the best course of action to be taken by the robot."""
        if secs is None: secs = self._default_plan_secs

        ## Get Current Perceived States

        current_front_cam_state = self._perception.front_camera_state
        current_left_cam_state = self._perception.left_camera_state
        current_right_cam_state = self._perception.right_camera_state
        current_scanner_state = self._perception.laser_scan_state
        current_gps_state = self._perception.gps_state
        current_cmd_vel = self._perception.cmd_vel_state

        ## Lateral Cams Line Detection

        closest_left_stake_line = self._calculate_detected_line(
            cam_state=current_left_cam_state,
            crop_fn=v.crop_lateral_image,
            detection_fn=v.detect_stake,
            reduce_fn=self._left_cam_line_reducer,
        )

        closest_left_plant_line = self._calculate_detected_line(
            cam_state=current_left_cam_state,
            crop_fn=v.crop_lateral_image,
            detection_fn=v.detect_plants,
            reduce_fn=self._left_cam_line_reducer,
        )

        closest_right_stake_line = self._calculate_detected_line(
            cam_state=current_right_cam_state,
            crop_fn=v.crop_lateral_image,
            detection_fn=v.detect_stake,
            reduce_fn=self._right_cam_line_reducer,
        )

        closest_right_plant_line = self._calculate_detected_line(
            cam_state=current_right_cam_state,
            crop_fn=v.crop_lateral_image,
            detection_fn=v.detect_plants,
            reduce_fn=self._right_cam_line_reducer,
        )

        ## Front Cam Line Detection

        closest_front_left_plant_line = self._calculate_detected_line(
            cam_state=current_front_cam_state,
            crop_fn=v.crop_front_image,
            detection_fn=v.detect_plants,
            reduce_fn=self._front_left_line_reducer,
        )

        closest_front_left_stake_line = self._calculate_detected_line(
            cam_state=current_front_cam_state,
            crop_fn=v.crop_front_image,
            detection_fn=v.detect_stake,
            reduce_fn=self._front_left_line_reducer,
        )

        closest_front_right_plant_line = self._calculate_detected_line(
            cam_state=current_front_cam_state,
            crop_fn=v.crop_front_image,
            detection_fn=v.detect_plants,
            reduce_fn=self._front_right_line_reducer,
        )

        closest_front_right_stake_line = self._calculate_detected_line(
            cam_state=current_front_cam_state,
            crop_fn=v.crop_front_image,
            detection_fn=v.detect_stake,
            reduce_fn=self._front_right_line_reducer,
        )

        if current_front_cam_state is not None:
            frame = self._bridge.imgmsg_to_cv2(current_front_cam_state)
            image = frame

            if closest_front_left_plant_line and closest_right_plant_line:
                front_plant_lines = np.hstack((closest_front_left_plant_line, closest_front_right_plant_line))
                image = v.draw_lines_on_image(image, front_plant_lines.reshape(-1, 4), (0, 10, 200))

            if closest_front_left_stake_line and closest_front_right_stake_line:
                front_stake_lines = np.hstack((closest_front_left_stake_line, closest_front_right_stake_line))
                image = v.draw_lines_on_image(image, front_stake_lines.reshape(-1, 4), (150, 10, 5))

            cv2.imshow('camera', image)
            cv2.waitKey(1)

        #TODO: proccess the information and return the best control strategy
        return UcvSimpleActionPlan(x=0, theta=0, secs=0)
