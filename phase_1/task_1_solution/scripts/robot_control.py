import rospy
import time

from geometry_msgs.msg import Twist


class UcvRobotControl:
    def __init__(self, publishing_rate_in_hz = 10, debug=False):
        self.rate = rospy.Rate(publishing_rate_in_hz)
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self._debug = debug

    @property
    def cmd_vel_pub(self):
        return self._cmd_vel_pub

    def set_publish_rate(self, hz):
        self.rate = rospy.Rate(hz)

    def publish_to_cmd_vel(self, twist, secs=0):
        now = time.time()
        while secs == 0 or (time.time() - now) < secs:
            if self._debug is True:
                rospy.loginfo(
                    'Publishing cmd vel: x = %f, z = %f'
                    % (twist.linear.x, twist.angular.z)
                )
            self._cmd_vel_pub.publish(twist)
            if secs == 0: # should execute only once
                break
            self.rate.sleep()

    def stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.publish_to_cmd_vel(cmd, 0)

    def move_regular(self, x=0, theta=0, secs=1):
        twist = Twist()
        twist.linear.x = x
        twist.angular.z = theta
        self.publish_to_cmd_vel(twist, secs)

    def execute_plan(self, plan):
        self.publish_to_cmd_vel(plan.to_twist(), plan.secs)
