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
from collections import deque

from helpers import (
    ForgetfulMemory,
    BasicQueue
)


class UcvSimpleActionPlan:
    def __init__(self, x, theta, secs=0, debug=False):
        self.debug = debug
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
    def __init__(self, control, perception, default_plan_timeslot_in_secs=1, debug=False):
        self.debug = debug

        self._control = control
        self._perception = perception
        self._bridge = CvBridge()

        self._last_actions_memory = ForgetfulMemory(5)
        self._next_actions_queue = BasicQueue()

        self._default_plan_secs = default_plan_timeslot_in_secs

        self._left_cam_line_reducer = r.define_line_reducer_on_point(
            point=cons.CROPPED_LATERAL_LEFT_VISION_POINT,
        )

        self._right_cam_line_reducer = r.define_line_reducer_on_point(
            point=cons.CROPPED_LATERAL_RIGHT_VISION_POINT,
        )

        self._front_left_line_reducer = r.define_line_reducer_on_point(
            point=cons.FRONT_VISION_LEFT_POINT
        )

        self._front_right_line_reducer = r.define_line_reducer_on_point(
            point=cons.FRONT_VISION_RIGHT_POINT
        )

    @property
    def _has_enqueued_actions(self):
        return not self._next_actions_queue.empty()

    def _memoize_action(self, action, only_non_zero=True):
        """Add action to the memory. If `only_non_zero` is True then, an action
        will onle be stored in the actions memory if it doesn't have it's vars with
        the values to zero."""
        if only_non_zero is True and action.x == 0 and action.theta == 0:
            return
        self._last_actions_memory.add(action)

    def _resolve_enqueued_actions(self):
        """Returns the next action in the queue, removing it from the structure."""
        if not self._next_actions_queue.empty():
            return self._next_actions_queue.dequeue()
        return UcvRobotPlanner(x=0, theta=0, secs=0)

    def _calculate_detected_lines(self, cam_state, *, detection_fn, reduce_fn=None, crop_fn=None, mask_fn=None, mask=None):
        lines = None
        if cam_state is not None:
            image = self._bridge.imgmsg_to_cv2(cam_state)

            if crop_fn is not None:
                image = crop_fn(image)

            if mask_fn is not None and mask is not None:
                image = mask_fn(image, mask=mask)

            image = detection_fn(image, image_is_hsv=False)
            lines = v.hough_lines(image, image_is_canny=False)

            if lines is not None and reduce_fn is not None:
                lines = reduce(reduce_fn, lines.reshape(-1, 4))
        return lines

    def plan(self, secs=None):
        """Analyse the information from the perception mechanisms
        and determine the best course of action to be taken by the robot."""
        if self._has_enqueued_actions is True:
            return self._resolve_enqueued_actions()

        ## Get Current Perceived States

        current_front_cam_state = self._perception.front_camera_state
        current_left_cam_state = self._perception.left_camera_state
        current_right_cam_state = self._perception.right_camera_state
        current_scanner_state = self._perception.laser_scan_state
        current_gps_state = self._perception.gps_state
        current_cmd_vel = self._perception.cmd_vel_state

        ## Lateral Cams Line Detection

        closest_left_stake_line = self._calculate_detected_lines(
            cam_state=current_left_cam_state,
            crop_fn=v.crop_lateral_image,
            detection_fn=v.detect_stake,
            reduce_fn=self._left_cam_line_reducer,
        )

        closest_left_plant_line = self._calculate_detected_lines(
            cam_state=current_left_cam_state,
            crop_fn=v.crop_lateral_image,
            detection_fn=v.detect_plants,
            reduce_fn=self._left_cam_line_reducer,
        )

        closest_right_stake_line = self._calculate_detected_lines(
            cam_state=current_right_cam_state,
            crop_fn=v.crop_lateral_image,
            detection_fn=v.detect_stake,
            reduce_fn=self._right_cam_line_reducer,
        )

        closest_right_plant_line = self._calculate_detected_lines(
            cam_state=current_right_cam_state,
            crop_fn=v.crop_lateral_image,
            detection_fn=v.detect_plants,
            reduce_fn=self._right_cam_line_reducer,
        )

        ## Front Cam Line Detection

        closest_front_left_plant_line = self._calculate_detected_lines(
            cam_state=current_front_cam_state,
            crop_fn=v.crop_front_image,
            detection_fn=v.detect_plants,
            reduce_fn=self._front_left_line_reducer,
            mask_fn=v.mask_image,
            mask=cons.FRONT_MASK_01,
        )

        closest_front_left_stake_line = self._calculate_detected_lines(
            cam_state=current_front_cam_state,
            crop_fn=v.crop_front_image,
            detection_fn=v.detect_stake,
            reduce_fn=self._front_left_line_reducer,
            mask_fn=v.mask_image,
            mask=cons.FRONT_MASK_01,
        )

        closest_front_right_plant_line = self._calculate_detected_lines(
            cam_state=current_front_cam_state,
            crop_fn=v.crop_front_image,
            detection_fn=v.detect_plants,
            reduce_fn=self._front_right_line_reducer,
            mask_fn=v.mask_image,
            mask=cons.FRONT_MASK_01,
        )

        closest_front_right_stake_line = self._calculate_detected_lines(
            cam_state=current_front_cam_state,
            crop_fn=v.crop_front_image,
            detection_fn=v.detect_stake,
            reduce_fn=self._front_right_line_reducer,
            mask_fn=v.mask_image,
            mask=cons.FRONT_MASK_01,
        )

        if current_front_cam_state is not None:
            frame = self._bridge.imgmsg_to_cv2(current_front_cam_state)
            image = frame

            if closest_front_left_plant_line is not None and closest_front_right_plant_line is not None:
                front_plant_lines = np.hstack((closest_front_left_plant_line, closest_front_right_plant_line))
                image = v.draw_lines_on_image(image, front_plant_lines.reshape(-1, 4), (0, 10, 200))

                theta = r.theta_front_transfer_function(
                    closest_front_left_line=closest_front_left_plant_line,
                    closest_front_right_line=closest_front_right_plant_line,
                    left_ref_point=cons.FRONT_VISION_LEFT_POINT,
                    right_ref_point=cons.FRONT_VISION_RIGHT_POINT,
                )

                secs = self._default_plan_secs if secs is None else secs

                self._next_actions_queue.enqueue(UcvSimpleActionPlan(x=0, theta=theta, secs=0))
                self._next_actions_queue.enqueue(UcvSimpleActionPlan(x=0.1, theta=0.0, secs=secs))
                self._next_actions_queue.enqueue(UcvSimpleActionPlan(x=0, theta=-theta, secs=0))
                self._next_actions_queue.enqueue(UcvSimpleActionPlan(x=0.2, theta=0, secs=secs))
                self._next_actions_queue.enqueue(UcvSimpleActionPlan(x=0, theta=0))
                return self._resolve_enqueued_actions()

            # cv2.imshow('camera', image)
            # cv2.waitKey(1)


        #TODO: proccess the information and return the best control strategy
        return UcvSimpleActionPlan(x=0, theta=0, secs=0)
