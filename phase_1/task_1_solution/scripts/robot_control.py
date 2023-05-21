import rospy

from geometry_msgs.msg import Twist

from robot_sensor import UcvSensorType


class UcvRobotControl:
    def __init__(self, robot_sensor, default_publishing_rate = 10):
        self._sensor = robot_sensor
        self.default_publishing_rate = default_publishing_rate

        self._sensor.register_state_update_callback(
            sensor_type = UcvSensorType.CAM_LEFT,
            callback = self.on_left_camera_state_update,
        )
        self._sensor.register_state_update_callback(
            sensor_type = UcvSensorType.CAM_RIGHT,
            callback = self.on_right_camera_state_update,
        )
        self._sensor.register_state_update_callback(
            sensor_type = UcvSensorType.CAM_FRONT,
            callback = self.on_front_camera_state_update,
        )
        self._sensor.register_state_update_callback(
            sensor_type = UcvSensorType.GPS,
            callback = self.on_gps_state_update,
        )

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

    def on_front_camera_state_update(self, data):
        rospy.loginfo('Robot Control: front camera state update action')

    def on_left_camera_state_update(self, data):
        rospy.loginfo('Robot Control: left camera state update action')

    def on_right_camera_state_update(self, data):
        rospy.loginfo('Robot Control: right camera state update action')

    def on_gps_state_update(self, data):
        rospy.loginfo('Robot Control: gps state update action')

