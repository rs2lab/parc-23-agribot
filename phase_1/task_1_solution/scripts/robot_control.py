import rospy

from geometry_msgs.msg import Twist


class UcvRobotControl:
    def __init__(self, publishing_rate_in_hz = 10):
        self.rate = rospy.Rate(publishing_rate_in_hz)
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

    @property
    def cmd_vel_pub(self):
        return self._cmd_vel_pub

    def publish_cmd_vel(twist_move_cmd):
        self._cmd_vel_pub.publish(twist_move_cmd)
        self.rate.sleep()
