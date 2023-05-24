import rospy
import time

from geometry_msgs.msg import Twist


class UcvRobotControl:
    def __init__(self, publishing_rate_in_hz = 10):
        self.rate = rospy.Rate(publishing_rate_in_hz)
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

    def change_publishing_rate_to(self, hz):
        self.rate = rospy.Rate(hz)

    @property
    def cmd_vel_pub(self):
        return self._cmd_vel_pub

    def publish_to_cmd_vel(self, twist, secs):
        now = time.time()
        while (time.time() - now) < secs:
            self._cmd_vel_pub.publish(twist)
            self.rate.sleep()

    def stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.publish_to_cmd_vel(cmd)

    def move_regular(self, x=0, theta=0, secs=1):
        twist = Twist()
        twist.linear.x = x
        twist.angular.z = theta
        self.publish_to_cmd_vel(twist, secs)
