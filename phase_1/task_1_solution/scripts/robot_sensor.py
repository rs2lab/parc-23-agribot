import rospy
import enum

from sensor_msgs.msg import (
    Image,
    NavSatFix,
    LaserScan
)


class UcvSensorType(enum.Enum):
    CAM_LEFT = 'left_camera'
    CAM_RIGHT = 'right_camera'
    CAM_FRONT = 'front_camera'
    LASER_SCAN = 'laser_scanner'
    GPS = 'gps'


class UcvRobotSensor:
    def __init__(self, debug = False):
        self.debug = debug

        self._left_camera_state = None
        self._right_camera_state = None
        self._front_camera_state = None
        self.laser_scan = None
        self._laser_scan = None

        rospy.Subscriber('/left_camera/image_raw', Image, self._left_camera_state_update_handler)
        rospy.Subscriber('/right_camera/image_raw', Image, self._right_camera_state_update_handler)
        rospy.Subscriber('/camera/image_raw', Image, self._front_camera_state_update_handler)
        rospy.Subscriber('/gps/fix', NavSatFix, self._gps_state_update_handler)
        rospy.Subscriber('/scan', LaserScan, self._laser_scan_state_update_handler)

        self._state_update_callback_registry = {
            UcvSensorType.CAM_LEFT: [],
            UcvSensorType.CAM_RIGHT: [],
            UcvSensorType.CAM_FRONT: [],
            UcvSensorType.GPS: [],
            UcvSensorType.LASER_SCAN: [],
        }

    def register_state_update_callback(self, sensor_type, callback):
        if sensor_type in self._state_update_callback_registry:
            self._state_update_callback_registry[sensor_type].append(callback)
            return True
        return False

    def _trigger_callbacks(self, sensor_type, data):
        if sensor_type in self._state_update_callback_registry:
            callbacks = self._state_update_callback_registry[sensor_type]
            if self.debug is True:
                rospy.loginfo(f'Calling {len(callbacks)} callback for {sensor_type.value} sensor state update')
            for callback in callbacks:
                callback(data)

    @property
    def left_camera_state(self):
        return self._left_camera_state

    @property
    def right_camera_state(self):
        return self._right_camera_state

    @property
    def front_camera_state(self):
        return self._front_camera_state

    @property
    def gps_state(self):
        return self._gps_state

    @property
    def laser_scan(self):
        return self._laser_scan

    def _left_camera_state_update_handler(self, data):
        self._left_camera_state = data
        self._trigger_callbacks(UcvSensorType.CAM_LEFT, data)

    def _right_camera_state_update_handler(self, data):
        self._right_camera_state = data
        self._trigger_callbacks(UcvSensorType.CAM_RIGHT, data)

    def _front_camera_state_update_handler(self, data):
        self._front_camera_state = data
        self._trigger_callbacks(UcvSensorType.CAM_FRONT, data)

    def _gps_state_update_handler(self, data):
        self._gps_state = data
        self._trigger_callbacks(UcvSensorType.GPS, data)

    def _laser_scan_state_update_handler(self, data):
        self._laser_scan = data
        self._trigger_callbacks(UcvSensorType.LASER_SCAN, data)