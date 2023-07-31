import rospy
import numpy as np

from .utils import ruler as r

from .perceiver import AgribotPerceiver
from .utils import SteppedAction, SingleStepStopAction, Action
from .utils import ForgetfulMemory, BasicQueue, RotationType


HYPOTHETICAL_X_DIST = 126


class AgribotPlanner:
    def __init__(self, perception: AgribotPerceiver) -> None:
        self._percept = perception
        self._last_actions_memory = ForgetfulMemory()
        self._next_actions_queue = BasicQueue()
        self._has_turned_first_row = False

    @property
    def _has_enqueued_actions(self) -> bool:
        return not self._next_actions_queue.empty()

    def _resolve_enqueued_action(self) -> Action:
        """Returns the next action in the queue, removing it from the structure."""
        if self._has_enqueued_actions:
            return self._next_actions_queue.dequeue()
        return None

    def enqueue_action(self, action: Action) -> None:
        """Add a new action to the action queue."""
        self._next_actions_queue.enqueue(action)

    def _process_env_perception(self) -> dict:
        snapshot = self._percept.snapshot()

        snapshot['laser_scan_angles'] = r.laser_angles(snapshot['laser_scan_state'])
        snapshot['front_theta'] = r.calculate_front_theta(**snapshot)

        if 'front_cam_state' in snapshot and snapshot['front_cam_state'] is not None:
            # image = cv2.imwrite('image.jpg', v.imgmsg_to_cv2(snapshot['front_cam_state']))
            rospy.logdebug(f'front_theta = {snapshot["front_theta"]}')

        return snapshot

    def plan_action(self) -> Action:
        """Analyse the information from the environment and determine the
        best course of action to be taken by the robot."""
        if self._has_enqueued_actions:
            return self._resolve_enqueued_action()

        kwargs = self._process_env_perception()

        if (finish := self._finish_navigation(**kwargs)) is not None:
            return finish
        elif (turn := self._make_a_turn(**kwargs)) is not None:
            self._percept.reset_cummulative_moves()
            return turn
        return self._move_forward(**kwargs)

    def _make_a_turn(self, **kwargs) -> Action:
        # if self._percept.cum_lin_x > HYPOTHETICAL_X_DIST:
        #     if not self._last_actions_memory.empty():
        #         theta_dev = -np.mean(self._last_actions_memory.all())
        #         rospy.logdebug(f'Applying theta dev recovery')
        #         self.enqueue_action(SteppedAction(x=0, theta=theta_dev, steps=1))

        #     direction = RotationType.CLOCKWISE if not self._has_turned_first_row else RotationType.ANTICLOCKWISE
        #     turn_ind = direction.value
        #     side = 'left' if direction == RotationType.ANTICLOCKWISE else 'right'
        #     rospy.logdebug(f'Time to make a turn to the {side} side!')

        #     self.enqueue_action(SingleStepStopAction())
        #     self.enqueue_action(SteppedAction(x=0, theta=turn_ind * np.pi * 0.1, steps=18))
        #     self.enqueue_action(SingleStepStopAction())
        #     self.enqueue_action(SteppedAction(x=0.6, theta=0, steps=20))
        #     self.enqueue_action(SingleStepStopAction())
        #     self.enqueue_action(SteppedAction(x=0, theta=turn_ind * np.pi * 0.1, steps=18))
        #     self.enqueue_action(SingleStepStopAction())
        #     self.enqueue_action(SteppedAction(x=0.2, theta=0, steps=10))

        #     self.enqueue_action(SingleStepStopAction(on_finished_cb=self._percept.reset_cummulative_moves))

        #     self._has_turned_first_row = True

        #     return self._resolve_enqueued_action()
        return None # TODO

    def _move_forward(self, front_theta, **kwargs) -> Action:
        last_theta_alpha = self._last_actions_memory.last()
        last_theta = last_theta_alpha[0] if last_theta_alpha is not None else None
        theta = r.theta_weighted_sum(front_theta=front_theta, last_theta=last_theta)
        alpha = r.alpha_theta(theta, last_theta=last_theta)
        scale = 0.1 ** np.abs(front_theta / 2)

        self._last_actions_memory.add((theta, alpha))

        rospy.logdebug(f'theta = {theta}, alpha = {alpha}')

        self.enqueue_action(SteppedAction(x=0.65 * scale, theta=theta * 0.1, steps=10))
        self.enqueue_action(SingleStepStopAction())
        self.enqueue_action(SteppedAction(x=0.2 * scale, theta=alpha * 0.1, steps=10))
        self.enqueue_action(SingleStepStopAction())

        return self._resolve_enqueued_action()

    def _finish_navigation(self, **kwargs) -> Action:
        return None # TODO