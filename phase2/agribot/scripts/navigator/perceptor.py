import enum
import rospy

from types import FunctionType
from nav_msgs.msg import Odometry


class SensorType(enum.Enum):
    POINT_CLOUD = '/front/zed_nodelet_front/point_cloud/cloud_registered'
    ODOM = '/odom'
    LASER = '/scan'


class AgribotPerceptor:
    def __init__(self) -> None:
        self._point_cloud = None
        self._odom = None
        self._laser_scan = None

        self._state_update_cb_registry = {
            SensorType.POINT_CLOUD: [],
            SensorType.ODOM: [],
            SensorType.LASER: [],
        }

        rospy.Subscriber(SensorType.ODOM.value, Odometry, )

    def register_state_update_callback(self, sensor_type: SensorType, callback) -> None:
        """Add callbacks that will be called when the state of one the topics we listen to
        is changed."""
        if sensor_type in self._state_update_cb_registry:
            self._state_update_cb_registry[sensor_type].append(callback)
            return True
        return False

    def _trigger_callbacks(self, sensor_type, data):
        """This triggers all registered callbacks for the specific sensor."""
        if sensor_type in self._state_update_callback_registry:
            callbacks = self._state_update_callback_registry[sensor_type]
            for callback in callbacks:
                callback(data)

    @property
    def point_cloud(self):
        return self._point_cloud

    @property
    def odom(self):
        return self._odom

    @property
    def laser_scan(self):
        return self._laser_scan

    def _laser_scan_state_update_handler(self, data) -> None:
        self._laser_scan = data
        self._trigger_callbacks(SensorType.LASER, data)

    def _odom_state_update_handler(self, data) -> None:
        self._odom = data
        self._trigger_callbacks(SensorType.ODOM, data)

    def _point_cloud_state_update_handler(self, data) -> None:
        self._point_cloud = data
        self._trigger_callbacks(SensorType.POINT_CLOUD, data)
