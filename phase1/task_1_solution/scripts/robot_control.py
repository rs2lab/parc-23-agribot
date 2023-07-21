import rospy

from geometry_msgs.msg import Twist
from helpers import ZeroTwist
from time import time


class UcvRobotControl:
    def __init__(self, rate = 10, latch = False, queue_size = 10):
        self.rate = rospy.Rate(rate)

        self._cmd_vel_pub = rospy.Publisher(
            '/cmd_vel',
            Twist,
            queue_size=queue_size,
            latch=latch,
        )

    @property
    def cmd_vel_pub(self):
        return self._cmd_vel_pub

    def set_publish_rate(self, hz):
        """Set the publish rate `hz` in hertz."""
        self.rate = rospy.Rate(hz)

    def publish_cmd_vel(self, twist):
        """Publish a `twist` command to the command vel topic."""
        rospy.logdebug(
            'publishing cmd vel: linear.x = %f, angular.z = %f'
            % (twist.linear.x, twist.angular.z)
        )
        self.cmd_vel_pub.publish(twist)
        self.rate.sleep()

    def stop(self):
        """Sets all variables responsible for the movement of the
        robot to zero"""
        self.publish_cmd_vel(ZeroTwist())

    def move_regular(self, x=0, theta=0, secs=1):
        """Execute a simple move command in the robot.

        ### Params:
            - `x`: linear velocity in meters per seconds
            - `theta`: angular rotation in radians per seconds
            - `secs`: number of seconds to the command to be executed
        """
        twist = Twist()
        twist.linear.x = x
        twist.angular.z = theta

        start = time()
        while (time() - start) < secs:
            self.publish_cmd_vel(twist)

    def execute_plan(self, plan):
        """Execute a given plan."""
        plan.init()
        while plan.has_next_step():
            plan.consume_step(self.publish_cmd_vel)
        plan.finish()
