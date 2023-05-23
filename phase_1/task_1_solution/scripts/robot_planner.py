import rospy
import cv2

from robot_perception import UcvSensorType
from cv_bridge import CvBridge


class UcvRobotPlanner:
    def __init__(self, control, perception):
        self._control = control
        self._perception = perception
        self._bridge = CvBridge()

    def plan(self):
        """Analyse the information from the perception mechanisms
        and determine the best course of action to be taken by the robot."""
        pass # TODO

    def execute(self):
        """Execute the plan using the control mechanisms to achieve the goal."""
        if self._perception.front_camera_state is not None:
            frame = self._bridge.imgmsg_to_cv2(self._perception.front_camera_state)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            cv2.imshow('camera', frame)
            cv2.waitKey(1)