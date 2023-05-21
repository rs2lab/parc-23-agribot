import rospy

from robot_sensor import UcvRobotSensor
from robot_control import UcvRobotControl


class UcvRobotAgent:
    def __init__(self, node_id = '/ucv/task_1_solver/robot_agent'):
        rospy.init_node(node_id, anonymous = False)
        self.sensor = UcvRobotSensor(debug = True)
        self.control = UcvRobotControl(self._sensor)

    def run(self):
        rospy.spin()

