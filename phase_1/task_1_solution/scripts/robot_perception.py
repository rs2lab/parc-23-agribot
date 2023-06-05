import rospy
import enum
import ruler

import numpy as np

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

from helpers import BasicGeoPos


class UcvSensorType(enum.Enum):
    CAM_LEFT = '/left_camera/image_raw'
    CAM_RIGHT = '/right_camera/image_raw'
    CAM_FRONT = '/camera/image_raw'
    LASER_SCAN = '/scan'
    GPS = '/gps/fix'
    CMD_VEL = '/cmd_vel'


class UcvRobotPerception:
    def __init__(self, debug = False):
        self.debug = debug

        self._left_camera_state = None
        self._right_camera_state = None
        self._front_camera_state = None
        self._gps_state = None
        self._laser_scan_state = None
        self._cmd_vel_state = None

        self._goal_pos = None
        self._peg_1_pos = None
        self._peg_2_pos = None
        self._peg_3_pos = None
        self._peg_4_pos = None
        self._peg_5_pos = None
        self._peg_6_pos = None
        self._peg_7_pos = None
        self._peg_8_pos = None
        self._peg_A_pos = None
        self._peg_B_pos = None

        self._perceived_route = None

        self._state_update_callback_registry = {
            UcvSensorType.CAM_LEFT: [],
            UcvSensorType.CAM_RIGHT: [],
            UcvSensorType.CAM_FRONT: [],
            UcvSensorType.GPS: [self._capture_route_number],
            UcvSensorType.LASER_SCAN: [],
            UcvSensorType.CMD_VEL: [],
        }

        rospy.Subscriber(UcvSensorType.CAM_LEFT.value, Image, self._left_camera_state_update_handler)
        rospy.Subscriber(UcvSensorType.CAM_RIGHT.value, Image, self._right_camera_state_update_handler)
        rospy.Subscriber(UcvSensorType.CAM_FRONT.value, Image, self._front_camera_state_update_handler)
        rospy.Subscriber(UcvSensorType.GPS.value, NavSatFix, self._gps_state_update_handler)
        rospy.Subscriber(UcvSensorType.LASER_SCAN.value, LaserScan, self._laser_scan_state_update_handler)
        rospy.Subscriber(UcvSensorType.CMD_VEL.value, Twist, self._cmd_vel_state_update_handler)

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
    def first_gps_state(self):
        """The first captured gps state. Returns `sensor_msgs.msg.NavSatFix`

        Attributes:
            - header.seq: uint32
            - header.stamp: time
            - header.frame_id: string
            - latitude: float64
            - longitude: float64
            - altitude: float64
        """
        return self._first_gps_state

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
    def goal_pos(self):
        if self._goal_pos is None:
            self._goal_pos = BasicGeoPos(
                latitude=rospy.get_param('goal_latitude'),
                longitude=rospy.get_param('goal_longitude'),
            )
        return self._goal_pos

    def _get_peg_pos(self, idf):
        return BasicGeoPos(
            latitude=rospy.get_param('/peg_{}/latitude'.format(idf)),
            longitude=rospy.get_param('/peg_{}/longitude'.format(idf)),
        )

    @property
    def peg_1_pos(self):
        if self._peg_1_pos is None:
            self._peg_1_pos = self._get_peg_pos('01')
        return self._peg_1_pos

    @property
    def peg_2_pos(self):
        if self._peg_2_pos is None:
            self._peg_2_pos = self._get_peg_pos('02')
        return self._peg_2_pos

    @property
    def peg_3_pos(self):
        if self._peg_3_pos is None:
            self._peg_3_pos = self._get_peg_pos('03')
        return self._peg_3_pos

    @property
    def peg_4_pos(self):
        if self._peg_4_pos is None:
            self._peg_4_pos = self._get_peg_pos('04')
        return self._peg_4_pos


    @property
    def peg_5_pos(self):
        if self._peg_5_pos is None:
            self._peg_5_pos = self._get_peg_pos('05')
        return self._peg_5_pos

    @property
    def peg_6_pos(self):
        if self._peg_6_pos is None:
            self._peg_6_pos = self._get_peg_pos('06')
        return self._peg_6_pos

    @property
    def peg_7_pos(self):
        if self._peg_7_pos is None:
            self._peg_7_pos = self._get_peg_pos('07')
        return self._peg_7_pos

    @property
    def peg_8_pos(self):
        if self._peg_8_pos is None:
            self._peg_8_pos = self._get_peg_pos('08')
        return self._peg_8_pos

    @property
    def peg_A_pos(self):
        if self._peg_A_pos is None:
            self._peg_A_pos = self._get_peg_pos('A')
        return self._peg_A_pos

    @property
    def peg_B_pos(self):
        if self._peg_B_pos is None:
            self._peg_B_pos = self._get_peg_pos('B')
        return self._peg_B_pos

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
        if self._perceived_route is None:
            self._perceived_route = self._capture_route_number()
        self._trigger_callbacks(UcvSensorType.GPS, data)

    def _laser_scan_state_update_handler(self, data):
        self._laser_scan_state = data
        self._trigger_callbacks(UcvSensorType.LASER_SCAN, data)

    def _cmd_vel_state_update_handler(self, data):
        self._cmd_vel_state = data
        self._trigger_callbacks(UcvSensorType.CMD_VEL, data)

    def dist_to_peg_line(self, peg_a, peg_b):
        dist = None
        if self.gps_state is not None and peg_a is not None and peg_b is not None:
            robot_pos = np.array((self.gps_state.latitude, self.gps_state.longitude))
            peg_line_pos = np.array((peg_a.lat, peg_a.lon, peg_b.lat, peg_b.lon))
            dist = ruler.line_dist_to_point(peg_line_pos, robot_pos)
        return dist

    def dist_to_peg_line_1(self):
        return self.dist_to_peg_line(
            peg_a=self.peg_1_pos,
            peg_b=self.peg_4_pos,
        )

    def dist_to_peg_line_2(self):
        return self.dist_to_peg_line(
            peg_a=self.peg_2_pos,
            peg_b=self.peg_3_pos,
        )

    def dist_to_peg_line_3(self):
        return self.dist_to_peg_line(
            peg_a=self.peg_3_pos,
            peg_b=self.peg_6_pos,
        )

    def dist_to_peg_line_4(self):
        return self.dist_to_peg_line(
            peg_a=self.peg_4_pos,
            peg_b=self.peg_5_pos,
        )

    def dist_to_peg_line_5(self):
        return self.dist_to_peg_line(
            peg_a=self.peg_6_pos,
            peg_b=self.peg_7_pos,
        )

    def dist_to_peg_line_6(self):
        return self.dist_to_peg_line(
            peg_a=self.peg_5_pos,
            peg_b=self.peg_8_pos,
        )

    @property
    def route_perceived(self):
        """Returns the number of the route the robot will have to follow.
        It can be 1, 2, or 3.
        """
        return self._perceived_route

    def _capture_route_number(self):
        dl_1 = self.dist_to_peg_line_1()
        dl_5 = self.dist_to_peg_line_5()
        dl_6 = self.dist_to_peg_line_6()

        route_num = None
        if dl_1 is not None and dl_5 is not None and dl_6 is not None:
            min_dist = min(dl_1, dl_5, dl_6)
            if min_dist == dl_1:
                route_num = 1
            elif min_dist == dl_5:
                route_num = 2
            elif min_dist == dl_6:
                route_num == 3
        return route_num
