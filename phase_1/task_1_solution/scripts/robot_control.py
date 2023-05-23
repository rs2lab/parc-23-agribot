import rospy

from geometry_msgs.msg import Twist

from robot_perception import UcvSensorType


class UcvRobotControl:
    def __init__(self):
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

    @property
    def cmd_vel_pub(self):
        return self._cmd_vel_pub

    def publish_cmd_vel(twist_move_cmd):
        self._cmd_vel_pub.publish(twist_move_cmd)
