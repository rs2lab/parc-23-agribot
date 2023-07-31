from time import time
from geometry_msgs.msg import Twist


class Action:
    """Generic action class. This is an abstract class, hence, does not represent the
    concrete representation of the action to be taken by the robot."""
    def __init__(self, x, theta, on_finished_cb=None) -> None:
        self._on_finished_cb = on_finished_cb
        self._x = x
        self._theta = theta

    @property
    def x(self):
        return self._x

    @property
    def theta(self):
        return self._theta

    def to_twist(self) -> Twist:
        twist = Twist()
        twist.linear.x = self.x
        twist.angular.z = self.theta
        return twist

    ## NOTE: Should be implemented downstream
    def init(self) -> None:
        """Initializes all variables needed to execute the action."""
        pass

    ## NOTE: Should be implemented downstream
    def has_next_step(self) -> bool:
        """Returns True if this action has a next step to be executed."""
        return False

    def consume_step(self, fn) -> None:
        """Consumes one step of the action. Receives a function `fn`
        that receives a twist command as a param."""
        fn(self.to_twist())

    def finish(self) -> None:
        """Finishes the execution of the action."""
        if self._on_finished_cb is not None:
            self._on_finished_cb()


class TemporalAction(Action):
    def __init__(self, x, theta, secs=0, on_finished_cb=None) -> None:
        super().__init__(x, theta, on_finished_cb=on_finished_cb)
        self._secs = secs
        self._start_time = None
        self._secs_spent = None

    @property
    def secs(self):
        return self._secs

    @classmethod
    def from_twist(cls, twist, secs=0) -> None:
        return TemporalAction(
            theta=twist.angular.z,
            x=twist.linear.x,
            secs=secs,
        )

    def init(self) -> None:
        self._start_time = time()
        self._secs_spent = 0

    def has_next_step(self) -> bool:
        self._secs_spent = time() - self._start_time
        return self._secs_spent < self._secs


class SteppedAction(Action):
    def __init__(self, x, theta, steps=1, on_finished_cb=None) -> None:
        super().__init__(x, theta, on_finished_cb=on_finished_cb)
        self._steps = steps
        self._current_step = None

    @property
    def steps(self):
        return self._steps

    @classmethod
    def from_twist(cls, twist, steps=1) -> None:
        return TemporalAction(
            theta=twist.angular.z,
            x=twist.linear.x,
            steps=steps,
        )

    def init(self) -> None:
        self._current_step = 0

    def has_next_step(self) -> bool:
        return self._current_step < self.steps

    def consume_step(self, fn) -> None:
        self._current_step += 1
        super().consume_step(fn)


class EternalStoppingAction(Action):
    """This will make the vehicle stop forever while the program
    is running."""
    def __init__(self) -> None:
        super().__init__(0, 0, on_finished_cb=None)

    def has_next_step(self) -> bool:
        return True


class SingleStepStopAction(SteppedAction):
    def __init__(self, on_finished_cb=None) -> None:
        super().__init__(x=0, theta=0, steps=1, on_finished_cb=on_finished_cb)
