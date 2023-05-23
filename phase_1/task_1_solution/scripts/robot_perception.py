import rospy
import enum

from sensor_msgs.msg import (
    Image,
    NavSatFix,
    LaserScan
)
from geometry_msgs.msg import Twist


class UcvSensorType(enum.Enum):
    CAM_LEFT = 'left_camera'
    CAM_RIGHT = 'right_camera'
    CAM_FRONT = 'front_camera'
    LASER_SCAN = 'laser_scanner'
    GPS = 'gps'
    CMD_VEL = 'cmd_vel'


class UcvRobotPerception:
    def __init__(self, debug = False):
        self.debug = debug

        self._left_camera_state = None
        self._right_camera_state = None
        self._front_camera_state = None
        self._gps_state = None
        self._laser_scan_state = None
        self._cmd_vel_state = None

        self._state_update_callback_registry = {
            UcvSensorType.CAM_LEFT: [],
            UcvSensorType.CAM_RIGHT: [],
            UcvSensorType.CAM_FRONT: [],
            UcvSensorType.GPS: [],
            UcvSensorType.LASER_SCAN: [],
            UcvSensorType.CMD_VEL: [],
        }

        rospy.Subscriber('/left_camera/image_raw', Image, self._left_camera_state_update_handler)
        rospy.Subscriber('/right_camera/image_raw', Image, self._right_camera_state_update_handler)
        rospy.Subscriber('/camera/image_raw', Image, self._front_camera_state_update_handler)
        rospy.Subscriber('/gps/fix', NavSatFix, self._gps_state_update_handler)
        rospy.Subscriber('/scan', LaserScan, self._laser_scan_state_update_handler)
        rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_state_update_handler)

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
        """Returns `sensor_msgs.msg.Image`

        Attributes:
            - header.seq: uint32
            - header.stamp: time
            - header.frame_id: string
            - height: uint32
            - width: uint32
            - encoding: string
            - is_bigendian: uint8
            - step: uint32
            - data: uint8[]
        """
        return self._left_camera_state

    @property
    def right_camera_state(self):
        """Returns `sensor_msgs.msg.Image`

        Attributes:
            - header.seq: uint32
            - header.stamp: time
            - header.frame_id: string
            - height: uint32
            - width: uint32
            - encoding: string
            - is_bigendian: uint8
            - step: uint32
            - data: uint8[]
        """
        return self._right_camera_state

    @property
    def front_camera_state(self):
        """Returns `sensor_msgs.msg.Image`

        Attributes:
            - header.seq: uint32
            - header.stamp: time
            - header.frame_id: string
            - height: uint32
            - width: uint32
            - encoding: string
            - is_bigendian: uint8
            - step: uint32
            - data: uint8[]
        """
        return self._front_camera_state

    @property
    def gps_state(self):
        """Returns `sensor_msgs.msg.NavSatFix`

        Attributes:
            - header.seq: uint32
            - header.stamp: time
            - header.frame_id: string
            - latitude: float64
            - longitude: float64
            - altitude: float64
        """
        return self._gps_state

    @property
    def laser_scan_state(self):
        """Returns `sensor_msgs.msg.LaserScan`

        Attributes:
            - header.seq: uint32
            - header.stamp: time
            - header.frame_id: string
            - angle_min: float32
            - angle_max: float32
            - angle_increment: float32
            - time_increment: float32
            - scan_time: float32
            - range_min: float32
            - range_max: float32
            - ranges: float32[]
            - intensities: float32[]
        """
        return self._laser_scan_state

    @property
    def cmd_vel_state(self):
        """Returns `geometry_msgs.msg.Twist`

        Attributes:
            - linear.x: float64
            - linear.y: float64
            - linear.z: float64
            - angular.x: float64
            - angular.y: float64
            - angular.z: float64
        """
        return self._cmd_vel_state

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
        self._laser_scan_state = data
        self._trigger_callbacks(UcvSensorType.LASER_SCAN, data)

    def _cmd_vel_state_update_handler(self, data):
        self._cmd_vel_state = data
        self._trigger_callbacks(UcvSensorType.CMD_VEL, data)
