import rospy
import time

from geometry_msgs.msg import Twist
from utils.helpers import TwistZero
from utils.constants import CMD_VEL_TOPIC
from utils.action import Action


class AgribotController:
    def __init__(self, pub_rate: int = 10, **kwargs) -> None:
        self._pub_rate = 10
        self._rate = rospy.Rate(pub_rate)
        self._cmd_vel = rospy.Publisher(
            CMD_VEL_TOPIC,
            Twist,
            **kwargs
        )

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
        self._rate.sleep()

    def stop(self) -> None:
        """Stops the robot's movement."""
        self.pub_cmd_vel(TwistZero())

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
        action.init()
        while action.has_next_step():
            action.consume_step(self.pub_cmd_vel)
        action.finish()
