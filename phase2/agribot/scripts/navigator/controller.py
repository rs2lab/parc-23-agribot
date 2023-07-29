import rospy
import time

from geometry_msgs.msg import Twist
from .perceiver import AgribotPerceiver
from .utils import constants as cnst
from .utils import TwistZero
from .utils import Action


class AgribotController:
    def __init__(self, perception: AgribotPerceiver, pub_rate: int = 10, **kwargs) -> None:
        self._percept = perception
        self._pub_rate = pub_rate
        self._rate = rospy.Rate(self._pub_rate)
        self._received_stop_signal = False
        queue_size = kwargs['queue_size'] if 'queue_size' in kwargs else 10
        self._cmd_vel = rospy.Publisher(
            cnst.CMD_VEL_TOPIC,
            Twist,
            queue_size=queue_size,
            **kwargs
        )

    def activate_stop_signal(self) -> None:
        self._received_stop_signal = True

    def reset_stop_signal(self) -> None:
        self._received_stop_signal = False

    @property
    def cmd_vel(self) -> rospy.Publisher:
        return self.cmd_vel

    def pub_cmd_vel(self, twist: Twist) -> None:
        """Publish a `twist` command to the command vel topic."""
        rospy.logdebug(
            'publishing cmd vel: linear.x = %f, angular.z = %f'
            % (twist.linear.x, twist.angular.z)
        )
        self._cmd_vel.publish(twist)
        self._percept.record_twist_move(twist)
        self._rate.sleep()

    def stop(self) -> None:
        """Stops the robot's movement."""
        self.pub_cmd_vel(TwistZero())
        self.activate_stop_signal()

    def move_regular(self, x: float = 0, theta: float = 0, secs: float = 1) -> None:
        """Execute a simple move command in the robot.

        ### Params:
            - `x`: linear velocity in meters per seconds
            - `theta`: angular rotation in radians per seconds
            - `secs`: number of seconds to the command to be executed
        """
        twist = Twist()
        twist.linear.x = x
        twist.angular.z = theta

        start = time.time()
        while (time.time() - start) < secs:
            self.pub_cmd_vel(twist)

    def execute_action(self, action: Action) -> None:
        """Executes a given action."""
        self.reset_stop_signal()
        action.init()
        while action.has_next_step() and not self._received_stop_signal:
            action.consume_step(self.pub_cmd_vel)
        action.finish()
