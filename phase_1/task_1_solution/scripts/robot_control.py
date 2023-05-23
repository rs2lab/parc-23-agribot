import rospy

from geometry_msgs.msg import Twist

from robot_perception import UcvSensorType


# NOTE: be aware that we might have to face a 'race condition' (a concurrency problem)
#   if the state of the robot control object is accessed by the callbacks when the
#   sensors' states update, because two or more callbacks can be called almost simultaneously.


class UcvRobotControl:
    def __init__(self, robot_perception, default_publishing_rate = 10):
        self.default_publishing_rate = default_publishing_rate
        self._perception = robot_perception

        self._perception.register_state_update_callback(
            sensor_type = UcvSensorType.CAM_LEFT,
            callback = self.on_left_camera_state_update,
        )
        self._perception.register_state_update_callback(
            sensor_type = UcvSensorType.CAM_RIGHT,
            callback = self.on_right_camera_state_update,
        )
        self._perception.register_state_update_callback(
            sensor_type = UcvSensorType.CAM_FRONT,
            callback = self.on_front_camera_state_update,
        )
        self._perception.register_state_update_callback(
            sensor_type = UcvSensorType.GPS,
            callback = self.on_gps_state_update,
        )
        self._perception.register_state_update_callback(
            sensor_type = UcvSensorType.LASER_SCAN,
            callback = self.on_laser_scan_state_update,
        )
        self._perception.register_state_update_callback(
            sensor_type = UcvSensorType.CMD_VEL,
            callback = self.on_cmd_vel_state_update
        )

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

    def on_front_camera_state_update(self, data):
        pass # TODO: react to state update?

    def on_left_camera_state_update(self, data):
        pass # TODO: react to state update?

    def on_right_camera_state_update(self, data):
        pass # TODO: react to state update?

    def on_gps_state_update(self, data):
        pass # TODO: react to state update?

    def on_laser_scan_state_update(Self, data):
        pass # TODO: react to state update?

    def on_cmd_vel_state_update(self, data):
        # NOTE: Be careful here since this is updated mainly by us
        pass # TODO: react to state update?
