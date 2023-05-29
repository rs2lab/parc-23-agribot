import rospy
import cv2

import numpy as np
import vision as v
import constants as cons
import ruler as r

from geometry_msgs.msg import Twist
from functools import reduce

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

    def _detected_lines(self, cam_state, *, detection_fn, reduce_fn=None, crop_fn=None, mask_fn=None, mask=None):
        lines = None
        if cam_state is not None:
            image = v.imgmsg_to_cv2(cam_state)

            if crop_fn is not None:
                image = crop_fn(image)

            if mask_fn is not None and mask is not None:
                image = mask_fn(image, mask=mask)

            image = detection_fn(image, image_is_hsv=False)
            lines = v.hough_lines(image, image_is_canny=False)

            if lines is not None and reduce_fn is not None:
                lines = reduce(reduce_fn, lines.reshape(-1, 4))
        return lines

    def _calculate_lateral_theta(self, left_cam_state, right_cam_state):
        left_cam_plant_theta = r.lateral_shift_transfer_function(
            closest_left_line=self._detected_lines(
                cam_state=left_cam_state,
                crop_fn=v.crop_lateral_image,
                detection_fn=v.detect_plants,
                reduce_fn=self._left_cam_line_reducer,
            ),
            closest_right_line=self._detected_lines(
                cam_state=right_cam_state,
                crop_fn=v.crop_lateral_image,
                detection_fn=v.detect_plants,
                reduce_fn=self._right_cam_line_reducer,
            ),
        )

        left_cam_stake_theta = r.lateral_shift_transfer_function(
            closest_left_line=self._detected_lines(
                cam_state=left_cam_state,
                crop_fn=v.crop_lateral_image,
                detection_fn=v.detect_stake,
                reduce_fn=self._left_cam_line_reducer,
            ),
            closest_right_line=self._detected_lines(
                cam_state=right_cam_state,
                crop_fn=v.crop_lateral_image,
                detection_fn=v.detect_stake,
                reduce_fn=self._right_cam_line_reducer,
            ),
        )

        lateral_theta = left_cam_plant_theta + left_cam_stake_theta
        if left_cam_plant_theta != 0 and left_cam_stake_theta != 0:
            lateral_theta = lateral_theta / 2

        return lateral_theta

    def _calculate_front_theta(self, front_cam_state):
        front_cam_plant_theta = r.front_shift_transfer_function(
            closest_front_left_line=self._detected_lines(
                cam_state=front_cam_state,
                crop_fn=v.crop_front_image,
                detection_fn=v.detect_plants,
                reduce_fn=self._front_left_line_reducer,
                mask_fn=v.mask_image,
                mask=cons.FRONT_MASK_03,
            ),
            closest_front_right_line=self._detected_lines(
                cam_state=front_cam_state,
                crop_fn=v.crop_front_image,
                detection_fn=v.detect_plants,
                reduce_fn=self._front_right_line_reducer,
                mask_fn=v.mask_image,
                mask=cons.FRONT_MASK_03,
            ),
        )

        front_cam_stake_theta = r.front_shift_transfer_function(
            closest_front_left_line=self._detected_lines(
                cam_state=front_cam_state,
                crop_fn=v.crop_front_image,
                detection_fn=v.detect_stake,
                reduce_fn=self._front_left_line_reducer,
                mask_fn=v.mask_image,
                mask=cons.FRONT_MASK_03,
            ),
            closest_front_right_line=self._detected_lines(
                cam_state=front_cam_state,
                crop_fn=v.crop_front_image,
                detection_fn=v.detect_stake,
                reduce_fn=self._front_right_line_reducer,
                mask_fn=v.mask_image,
                mask=cons.FRONT_MASK_03,
            ),
        )

        front_theta = front_cam_plant_theta + front_cam_stake_theta
        if front_cam_plant_theta != 0 and front_cam_stake_theta != 0:
            front_theta = front_theta / 2

        return front_theta

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

        lateral_theta = self._calculate_lateral_theta(current_left_cam_state, current_right_cam_state)
        front_theta = self._calculate_front_theta(current_front_cam_state)

        theta = lateral_theta + front_theta

        if lateral_theta != 0 and front_theta != 0:
            theta = lateral_theta * 0.65 + front_theta * 0.35

        secs = self._default_plan_secs if secs is None else secs
        rate = self._control.rate.sleep_dur.nsecs / 10**9

        self._next_actions_queue.enqueue(UcvSimpleActionPlan(x=0.15, theta=theta * rate, secs=secs))
        self._next_actions_queue.enqueue(UcvSimpleActionPlan(x=0.0, theta=0.0, secs=0))
        self._next_actions_queue.enqueue(UcvSimpleActionPlan(x=0.125, theta=0.0, secs=secs))
        self._next_actions_queue.enqueue(UcvSimpleActionPlan(x=0.0, theta=0.0, secs=0))
        self._next_actions_queue.enqueue(UcvSimpleActionPlan(x=0.0, theta=-theta * rate, secs=secs))
        self._next_actions_queue.enqueue(UcvSimpleActionPlan(x=0, theta=0, secs=0))
        self._next_actions_queue.enqueue(UcvSimpleActionPlan(x=0.1, theta=0.0, secs=secs))
        self._next_actions_queue.enqueue(UcvSimpleActionPlan(x=0, theta=0, secs=0))

        return self._resolve_enqueued_actions()
