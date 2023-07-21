import rospy
import vision
import ruler

import numpy as np

from functools import reduce
from plan import UcvSteppedActionPlan

from constants import (
    SAFEST_BARN_TURN_DISTANCE,
    SAFEST_CAR_TURN_DISTANCE,
    CROPPED_LATERAL_LEFT_VISION_POINT,
    CROPPED_LATERAL_RIGHT_VISION_POINT,
    FRONT_VISION_LEFT_POINT,
    FRONT_VISION_RIGHT_POINT,
    FRONT_MASK_03,
)

from helpers import (
    ForgetfulMemory,
    BasicQueue,
    RotationType,
)


class UcvRobotPlanner:
    def __init__(self, control, perception):
        self._control = control
        self._perception = perception

        self._last_actions_memory = ForgetfulMemory()
        self._next_actions_queue = BasicQueue()

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

        self._cum_x = 0
        self._has_turned_first_row = False
        self._has_turned_second_row = False

    @property
    def _has_enqueued_actions(self):
        return not self._next_actions_queue.empty()

    def _resolve_enqueued_actions(self):
        """Returns the next action in the queue, removing it from the structure."""
        if not self._next_actions_queue.empty():
            planned_action = self._next_actions_queue.dequeue()
            return planned_action
        return UcvSteppedActionPlan(x=0, theta=0, steps=1)

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
            hidden=1.7,
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
            hidden=1.5,
        )

        self._lat_plant_theta = lateral_plant_theta
        lateral_theta = lateral_plant_theta + lateral_stake_theta / 2

        if lateral_plant_theta != 0 and lateral_stake_theta != 0:
            lateral_theta = lateral_plant_theta * 0.75 + lateral_stake_theta * 0.25

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

        front_theta = front_plant_theta + front_stake_theta / 2

        if front_plant_theta != 0 and front_stake_theta != 0:
            front_theta = front_plant_theta * 0.8 + front_stake_theta * 0.20

        return front_theta

    def _calculate_lfr(self, laser_scan_masked_ranges):
        """laser_fill_rate_in_the_view_angle""" # TODO: update docstring
        if laser_scan_masked_ranges is not None:
            return ruler.laser_front_fillup_rate(laser_scan_masked_ranges, mask_values=False)
        return None

    def _move_forward(self, front_theta, lateral_theta, **kwargs):
        last_theta_alpha = self._last_actions_memory.last()

        last_theta = last_theta_alpha[0] if last_theta_alpha is not None else None
        theta = ruler.theta_weighted_sum(lateral_theta=lateral_theta, front_theta=front_theta, last_theta=last_theta)
        alpha = ruler.alpha_theta(theta, last_theta=last_theta)

        self._last_actions_memory.add((theta, alpha))
        scale = 0.1 ** np.abs(front_theta - lateral_theta)

        self._cum_x += 0.8 * scale * 10 + 0.125 * scale * 10

        self.enqueue_action(UcvSteppedActionPlan(x=0.8 * scale, theta=theta * 0.1, steps=10))
        self.enqueue_action(UcvSteppedActionPlan(x=0.0, theta=0.0, steps=1))
        self.enqueue_action(UcvSteppedActionPlan(x=0.125 * scale, theta=alpha * 0.1, steps=10))
        self.enqueue_action(UcvSteppedActionPlan(x=0.0, theta=0.0, steps=1))

        return self._resolve_enqueued_actions()

    def _make_turn(self, laser_scan_state, laser_front_view_fill_rate, front_theta, closest_laser_dist, closest_laser_angle, **kwargs):
        """Analyse the laser and other sensors to see if it should turn left or right.
        Returns `(rho, theta)` or `None`"""
        if laser_scan_state is not None:
            rospy.logdebug(f'laser front view fill rate = {laser_front_view_fill_rate}')
            rospy.logdebug(f'closest laser front distance = {closest_laser_dist}')
            rospy.logdebug(f'closest laser front angle = {closest_laser_angle}')

            groute = self._perception.guess_route_number()
            switch = True if groute in (1, 3) else False
            if self._has_turned_first_row is True:
                switch = not switch

            safe_distance = SAFEST_BARN_TURN_DISTANCE if switch else SAFEST_CAR_TURN_DISTANCE

            rospy.logdebug('Cum X == {}'.format(self._cum_x))
            if front_theta == 0 and laser_front_view_fill_rate > 0.35 and self._cum_x > 124 and (self._cum_x > 127.5 or closest_laser_dist < safe_distance):
                self._cum_x = 0 # reset
                if self._has_turned_first_row and self._has_turned_second_row:
                    rospy.logdebug('Goal reach, has finished the route!')
                    return UcvSteppedActionPlan(x=0, theta=0, steps=1)
                if not self._last_actions_memory.empty():
                    theta_dev = -np.mean(self._last_actions_memory.all())

                    rospy.logdebug(f'applying theta dev recovery = {theta_dev}')

                    self._last_actions_memory.clear()
                    self.enqueue_action(UcvSteppedActionPlan(x=0, theta=theta_dev * 0.1, steps=10))

                direction = self._perception.guess_first_rotation_direction()
                rospy.logdebug("Route number: %i" % self._perception.guess_route_number())

                if self._has_turned_first_row is True:
                    self._has_turned_second_row = True
                    direction = RotationType.CLOCKWISE if direction == RotationType.ANTICLOCKWISE else RotationType.ANTICLOCKWISE 

                turn = direction.value
                side = 'left' if direction == RotationType.ANTICLOCKWISE else 'right'
                rospy.logdebug(f'Time to make a turn to the {side} side!')

                scale = 0.6 if self._has_turned_first_row else 1

                self.enqueue_action(UcvSteppedActionPlan(x=0.1, theta=0, steps=10))
                self.enqueue_action(UcvSteppedActionPlan(x=0, theta=0, steps=1))
                self.enqueue_action(UcvSteppedActionPlan(x=0, theta=turn * np.pi * 0.1, steps=18))
                self.enqueue_action(UcvSteppedActionPlan(x=0, theta=0, steps=1))
                self.enqueue_action(UcvSteppedActionPlan(x=0.95 * scale, theta=0, steps=20))
                self.enqueue_action(UcvSteppedActionPlan(x=0, theta=0, steps=1))
                self.enqueue_action(UcvSteppedActionPlan(x=0, theta=turn * np.pi * 0.1, steps=18))
                self.enqueue_action(UcvSteppedActionPlan(x=0, theta=0, steps=1))
                self.enqueue_action(UcvSteppedActionPlan(x=0.175, theta=0, steps=10))
                self.enqueue_action(UcvSteppedActionPlan(x=0, theta=0, steps=1))

                self._has_turned_first_row = True

                return self._resolve_enqueued_actions()
        return None

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

        kwargs['laser_scan_masked_ranges'] = ruler.mask_laser_scan(
            value=np.array(kwargs['laser_scan_state'].ranges) if kwargs['laser_scan_state'] else None
        )

        kwargs['laser_scan_angles'] = ruler.laser_angles(kwargs['laser_scan_state'])

        closest_laser_dist = None
        closest_laser_angle = None

        if kwargs['laser_scan_masked_ranges'] is not None:
            idx = np.argmin(kwargs['laser_scan_masked_ranges'])
            closest_laser_dist = kwargs['laser_scan_masked_ranges'][idx]
            closest_laser_angle = kwargs['laser_scan_angles'][idx]

        kwargs['closest_laser_dist'] = closest_laser_dist
        kwargs['closest_laser_angle'] = closest_laser_angle

        kwargs['lateral_theta'] = self._calculate_lateral_theta(
           left_cam_state=kwargs['left_cam_state'],
           right_cam_state=kwargs['right_cam_state']
        )

        kwargs['front_theta'] = self._calculate_front_theta(kwargs['front_cam_state'])
        kwargs['laser_front_view_fill_rate'] = self._calculate_lfr(kwargs['laser_scan_masked_ranges'])

        if (turn := self._make_turn(**kwargs)) is not None:
            return turn

        return self._move_forward(**kwargs)
