import enum
import rospy

from types import FunctionType
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan


class SensorType(enum.Enum):
    POINT_CLOUD = '/front/zed_nodelet_front/point_cloud/cloud_registered'
    FRONT_CAM = '/front/zed_nodelet_front/rgb_raw/image_raw_color'
    # LEFT_CAM = '' # TODO
    ODOM = '/odom'
    LASER = '/scan'


class AgribotPerceiver:
    def __init__(self) -> None:
        self._point_cloud = None
        self._laser_scan = None
        self._front_cam = None
        self._odom = None

        self._state_update_cb_registry = {
            SensorType.POINT_CLOUD: [],
            SensorType.FRONT_CAM: []
            SensorType.ODOM: [],
            SensorType.LASER: [],
        }

        rospy.Subscriber(SensorType.ODOM.value, Odometry, self._odom_state_update_handler)
        rospy.Subscriber(SensorType.LASER_SCAN.value, LaserScan, self._laser_scan_state_update_handler)
        rospy.Subscriber(SensorType.FRONT_CAM.value, Image, self._front_cam_state_update_handler)
        # rospy.Subscriber(SensorType.POINT_CLOUD.value, ) # TODO

    def register_state_update_callback(self, sensor_type: SensorType, callback) -> None:
        """Add callbacks that will be called when the state of one the topics we listen to
        is changed."""
        if sensor_type in self._state_update_cb_registry:
            self._state_update_cb_registry[sensor_type].append(callback)
            return True
        return False

    def _trigger_callbacks(self, sensor_type, data) -> None:
        """This triggers all registered callbacks for the specific sensor."""
        if sensor_type in self._state_update_callback_registry:
            callbacks = self._state_update_callback_registry[sensor_type]
            for callback in callbacks:
                callback(data)

    @property
    def point_cloud_state(self):
        return self._point_cloud

    @property
    def odom_state(self) -> Odometry:
        return self._odom

    @property
    def laser_scan_state(self) -> LaserScan:
        return self._laser_scan

    @property
    def front_cam_state(self) -> Image:
        return self._front_cam

    def _laser_scan_state_update_handler(self, data) -> None:
        self._laser_scan = data
        self._trigger_callbacks(SensorType.LASER, data)

    def _odom_state_update_handler(self, data) -> None:
        self._odom = data
        self._trigger_callbacks(SensorType.ODOM, data)

    def _point_cloud_state_update_handler(self, data) -> None:
        self._point_cloud = data
        self._trigger_callbacks(SensorType.POINT_CLOUD, data)

    def _front_cam_state_update_handler(self, data) -> None:
        self._front_cam = data
        self._trigger_callbacks(SensorType.FRONT_CAM, data)
