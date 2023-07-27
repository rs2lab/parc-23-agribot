from time import time
from geometry_msgs.msg import Twist


class Action:
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

    def init(self):
        """Initializes all variables needed to execute the action."""
        ## NOTE: Must be implemented downstream
        pass

    def has_next_step(self):
        """Returns True if this action has a next step to be executed."""
        ## NOTE: Should be implemented downstream
        return False

    def consume_step(self, fn):
        """Consumes one step of the action. Receives a function `fn`
        that receives a twist command as a param."""
        fn(self.to_twist())

    def finish(self):
        """Finishes the execution of the action."""
        ## NOTE: Should be implemented downstream
        pass


class TemporalAction(Action):
    def __init__(self, x, theta, secs=0):
        super().__init__(x, theta)
        self._secs = secs
        self._start_time = None
        self._secs_spent = None

    @property
    def secs(self):
        return self._secs

    @classmethod
    def from_twist(cls, twist, secs=0):
        return TemporalAction(
            theta=twist.angular.z,
            x=twist.linear.x,
            secs=secs,
        )

    def init(self):
        self._start_time = time()
        self._secs_spent = 0

    def has_next_step(self):
        self._secs_spent = time() - self._start_time
        return self._secs_spent < self._secs


class SteppedAction(Action):
    def __init__(self, x, theta, steps=1):
        super().__init__(x, theta)
        self._steps = steps
        self._current_step = None

    @property
    def steps(self):
        return self._steps

    @classmethod
    def from_twist(cls, twist, steps=1):
        return TemporalAction(
            theta=twist.angular.z,
            x=twist.linear.x,
            steps=steps,
        )

    def init(self):
        self._current_step = 0

    def has_next_step(self):
        return self._current_step < self.steps

    def consume_step(self, fn):
        self._current_step += 1
        super().consume_step(fn)


class EternalStoppingAction(Action):
    """This will make the vehicle stop forever while the program
    is running."""
    def __init__(self):
        super().__init__(0, 0)

    def has_next_step(self):
        return True
