import rospy
import enum

from sensor_msgs.msg import (
    Image,
    NavSatFix,
    LaserScan
)

from geometry_msgs.msg import (
    Twist,
    PoseWithCovarianceStamped,
    PoseStamped,
)


class UcvSensorType(enum.Enum):
    CAM_LEFT = '/left_camera/image_raw'
    CAM_RIGHT = '/right_camera/image_raw'
    CAM_FRONT = '/camera/image_raw'
    LASER_SCAN = '/scan'
    GPS = '/gps/fix'
    CMD_VEL = '/cmd_vel'
    GOAL = '/move_base_simple/goal'
    INITIAL_POSE = '/intialpose'


class UcvRobotPerception:
    def __init__(self, debug = False):
        self.debug = debug

        self._left_camera_state = None
        self._right_camera_state = None
        self._front_camera_state = None
        self._gps_state = None
        self._laser_scan_state = None
        self._cmd_vel_state = None
        self._initialpose_state = None
        self._goal_state = None

        self._state_update_callback_registry = {
            UcvSensorType.CAM_LEFT: [],
            UcvSensorType.CAM_RIGHT: [],
            UcvSensorType.CAM_FRONT: [],
            UcvSensorType.GPS: [],
            UcvSensorType.LASER_SCAN: [],
            UcvSensorType.CMD_VEL: [],
            UcvSensorType.INITIAL_POSE: [],
            UcvSensorType.GOAL: [],
        }

        rospy.Subscriber(UcvSensorType.CAM_LEFT.value, Image, self._left_camera_state_update_handler)
        rospy.Subscriber(UcvSensorType.CAM_RIGHT.value, Image, self._right_camera_state_update_handler)
        rospy.Subscriber(UcvSensorType.CAM_FRONT.value, Image, self._front_camera_state_update_handler)
        rospy.Subscriber(UcvSensorType.GPS.value, NavSatFix, self._gps_state_update_handler)
        rospy.Subscriber(UcvSensorType.LASER_SCAN.value, LaserScan, self._laser_scan_state_update_handler)
        rospy.Subscriber(UcvSensorType.CMD_VEL.value, Twist, self._cmd_vel_state_update_handler)
        rospy.Subscriber(UcvSensorType.INITIAL_POSE.value, PoseWithCovarianceStamped, self._initialpose_state_update_handler)
        rospy.Subscriber(UcvSensorType.GOAL.value, PoseStamped, self._goal_state_update_handler)

    def register_state_update_callback(self, sensor_type, callback):
        """Add a callback that will be called when the state of a sensor is updated."""
        if sensor_type in self._state_update_callback_registry:
            self._state_update_callback_registry[sensor_type].append(callback)
            return True
        return False

    def _trigger_callbacks(self, sensor_type, data):
        if sensor_type in self._state_update_callback_registry:
            callbacks = self._state_update_callback_registry[sensor_type]
            if self.debug is True:
                rospy.loginfo(f'Calling {len(callbacks)} callbacks for {sensor_type.value} sensor state update')
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

    @property
    def goal_state(self):
        """Returns `geometry_msgs.msg.PoseStamped`

        Attributes:
            - header.seq: uint32
            - header.time: stamp
            - header.frame_id: string
            - pose.position.x: float64
            - pose.position.y: float64
            - pose.position.z: float64
            - pose.orientation.x: float64
            - pose.orientation.y: float64
            - pose.orientation.z: float64
            - pose.orientation.w: float64
        """
        return self._goal_state

    @property
    def intialpose_state(self):
        """Returns `geometry_msgs.msg.PoseWithCovarianceStamped`

        Attributes:
            - header.seq: uint32
            - header.time: stamp
            - header.frame_id: string
            - pose.pose.position.x: float64
            - pose.pose.position.y: float64
            - pose.pose.position.z: float64
            - pose.pose.orientation.x: float64
            - pose.pose.orientation.y: float64
            - pose.pose.orientation.z: float64
            - pose.pose.orientation.w: float64
            - pose.convariance: float64[0..36]
        """
        return self._initialpose_state

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

    def _initialpose_state_update_handler(self, data):
        self._initialpose_state = data
        self._trigger_callbacks(UcvSensorType.INITIAL_POSE, data)

    def _goal_state_update_handler(self, data):
        self._goal_state = data
        self._trigger_callbacks(UcvSensorType.GOAL, data)
