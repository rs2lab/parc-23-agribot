import rospy
import vision
import ruler

import numpy as np

from geometry_msgs.msg import Twist
from functools import reduce

from constants import (
    CROPPED_LATERAL_LEFT_VISION_POINT,
    CROPPED_LATERAL_RIGHT_VISION_POINT,
    FRONT_VISION_LEFT_POINT,
    FRONT_VISION_RIGHT_POINT,
    FRONT_MASK_03,
    LASER_THETA
)

from helpers import (
    ForgetfulMemory,
    BasicQueue
)


class UcvActionPlan:
    def __init__(self, x, theta):
        self._x = x
        self._theta = theta

    @property
    def x(self):
        return self._x

    @property
    def theta(self):
        return self._theta

    def to_twist(self):
        twist = Twist()
        twist.linear.x = self.x
        twist.angular.z = self.theta
        return twist


class UcvTemporalActionPlan(UcvActionPlan):
    def __init__(self, x, theta, secs=0):
        super().__init__(x, theta)
        self._secs = secs

    @property
    def secs(self):
        return self._secs

    @classmethod
    def from_twist(cls, twist, secs=0):
        return UcvTemporalActionPlan(
            theta=twist.angular.z,
            x=twist.linear.x,
            secs=secs,
        )


class UcvSteppedActionPlan(UcvActionPlan):
    def __init__(self, x, theta, steps=1):
        super().__init__(x, theta)
        self._steps = steps

    @property
    def steps(self):
        return self._steps

    @classmethod
    def from_twist(cls, twist, steps=1):
        return UcvTemporalActionPlan(
            theta=twist.angular.z,
            x=twist.linear.x,
            steps=steps,
        )


class UcvRobotPlanner:
    def __init__(self, control, perception, default_plan_timeslot_in_secs=1, debug=False):
        self.debug = debug

        self._control = control
        self._perception = perception

        self._last_actions_memory = ForgetfulMemory(5)
        self._next_actions_queue = BasicQueue()

        self._default_plan_secs = default_plan_timeslot_in_secs

        self._left_cam_line_reducer = ruler.define_line_reducer_on_point(
            point=CROPPED_LATERAL_LEFT_VISION_POINT,
        )

        self._right_cam_line_reducer = ruler.define_line_reducer_on_point(
            point=CROPPED_LATERAL_RIGHT_VISION_POINT,
        )

        self._front_left_line_reducer = ruler.define_line_reducer_on_point(
            point=FRONT_VISION_LEFT_POINT
        )

        self._front_right_line_reducer = ruler.define_line_reducer_on_point(
            point=FRONT_VISION_RIGHT_POINT
        )

    @property
    def _has_enqueued_actions(self):
        return not self._next_actions_queue.empty()

    def _resolve_enqueued_actions(self):
        """Returns the next action in the queue, removing it from the structure."""
        if not self._next_actions_queue.empty():
            planned_action = self._next_actions_queue.dequeue()
            return planned_action
        return UcvSteppedActionPlan(x=0, theta=0, steps=0)

    def enqueue_action(self, action):
        """Add a new action to the action queue."""
        self._next_actions_queue.enqueue(action)

    def _detect_lines(self, image, *, detection_fn, reduce_fn=None, crop_fn=None, mask_fn=None, mask=None):
        lines = None
        if image is not None:
            if crop_fn is not None:
                image = crop_fn(image)

            if mask_fn is not None and mask is not None:
                image = mask_fn(image, mask=mask)

            image = detection_fn(image, image_is_hsv=False)
            lines = vision.hough_lines(image, image_is_canny=False)

            if lines is not None and reduce_fn is not None:
                lines = reduce(reduce_fn, lines.reshape(-1, 4))
        return lines

    def _calculate_lateral_theta(self, left_cam_state, right_cam_state):
        left_cam_image = None
        right_cam_image = None

        if left_cam_state is not None:
            left_cam_image = vision.imgmsg_to_cv2(left_cam_state)
            left_cam_image = vision.crop_lateral_image(left_cam_image)

        if right_cam_state is not None:
            right_cam_image = vision.imgmsg_to_cv2(right_cam_state)
            right_cam_image = vision.crop_lateral_image(right_cam_image)

        lateral_plant_theta = ruler.lateral_shift_transfer_function(
            closest_left_line=self._detect_lines(
                left_cam_image,
                detection_fn=vision.detect_plants,
                reduce_fn=self._left_cam_line_reducer,
            ),
            closest_right_line=self._detect_lines(
                right_cam_image,
                detection_fn=vision.detect_plants,
                reduce_fn=self._right_cam_line_reducer,
            ),
        )

        lateral_stake_theta = ruler.lateral_shift_transfer_function(
            closest_left_line=self._detect_lines(
                left_cam_image,
                detection_fn=vision.detect_stake,
                reduce_fn=self._left_cam_line_reducer,
            ),
            closest_right_line=self._detect_lines(
                right_cam_image,
                detection_fn=vision.detect_stake,
                reduce_fn=self._right_cam_line_reducer,
            ),
        )

        lateral_theta = lateral_plant_theta # + lateral_stake_theta

        if lateral_plant_theta != 0 and lateral_stake_theta != 0:
            lateral_theta = lateral_plant_theta * 0.65 + lateral_stake_theta * 0.35

        return lateral_theta

    def _calculate_front_theta(self, front_cam_state):
        front_cam_image = None

        if front_cam_state is not None:
            front_cam_image = vision.imgmsg_to_cv2(front_cam_state)
            front_cam_image = vision.crop_front_image(front_cam_image)
            front_cam_image = vision.mask_image(front_cam_image, mask=FRONT_MASK_03)

        front_plant_theta = ruler.front_shift_transfer_function(
            closest_front_left_line=self._detect_lines(
                front_cam_image,
                detection_fn=vision.detect_plants,
                reduce_fn=self._front_left_line_reducer,
            ),
            closest_front_right_line=self._detect_lines(
                front_cam_image,
                detection_fn=vision.detect_plants,
                reduce_fn=self._front_right_line_reducer,
            ),
        )

        front_stake_theta = ruler.front_shift_transfer_function(
            closest_front_left_line=self._detect_lines(
                front_cam_image,
                detection_fn=vision.detect_stake,
                reduce_fn=self._front_left_line_reducer,
            ),
            closest_front_right_line=self._detect_lines(
                front_cam_image,
                detection_fn=vision.detect_stake,
                reduce_fn=self._front_right_line_reducer,
            ),
        )

        front_theta = front_plant_theta # + front_stake_theta

        if front_plant_theta != 0 and front_stake_theta != 0:
            front_theta = front_plant_theta * 0.8 + front_stake_theta * 0.2

        return front_theta

    def _move_forward(self, front_cam_state, left_cam_state, right_cam_state, laser_scan_state, gps_state, **kwargs):
        lateral_theta = self._calculate_lateral_theta(left_cam_state, right_cam_state)
        front_theta = self._calculate_front_theta(front_cam_state)
        last_theta_alpha = self._last_actions_memory.last()

        last_theta = last_theta_alpha[0] if last_theta_alpha is not None else None
        theta = ruler.theta_weighted_sum(lateral_theta=lateral_theta, front_theta=front_theta, last_theta=last_theta)
        alpha = ruler.alpha_theta(theta, last_theta=last_theta)

        self._last_actions_memory.add((theta, alpha))

        self.enqueue_action(UcvSteppedActionPlan(x=0.2, theta=theta * 0.1, steps=10))
        self.enqueue_action(UcvSteppedActionPlan(x=0.0, theta=0.0, steps=1))
        #self.enqueue_action(UcvSteppedActionPlan(x=0.135, theta=0.0, steps=10))
        #self.enqueue_action(UcvSteppedActionPlan(x=0.0, theta=0.0, steps=1))
        self.enqueue_action(UcvSteppedActionPlan(x=0.1, theta=alpha * 0.1, steps=10))
        self.enqueue_action(UcvSteppedActionPlan(x=0.0, theta=0.0, steps=1))
        self.enqueue_action(UcvSteppedActionPlan(x=0.175, theta=0.0, steps=10))
        self.enqueue_action(UcvSteppedActionPlan(x=0.0, theta=0.0, steps=1))

        return self._resolve_enqueued_actions()

    def _should_make_turn(self, laser_scan_state, front_cam_state, left_cam_state, right_cam_state, **kwargs):
        """Analyse the laser and other sensors to see if it should turn left or right.
        Returns `(rho, theta)` or `None`"""
        # TODO: implement
        return None

    def _make_a_turn(self, direction, **kwargs):
        pass # TODO: implement

    def plan(self):
        """Analyse the information from the perception mechanisms
        and determine the best course of action to be taken by the robot."""
        if self._has_enqueued_actions:
            return self._resolve_enqueued_actions()

        kwargs = dict(
            front_cam_state = self._perception.front_camera_state,
            left_cam_state = self._perception.left_camera_state,
            right_cam_state = self._perception.right_camera_state,
            laser_scan_state = self._perception.laser_scan_state,
            gps_state = self._perception.gps_state,
        )

        if (direction := self._should_make_turn(**kwargs)) is not None:
            return self._make_a_turn(direction, **kwargs)
        return self._move_forward(**kwargs)
