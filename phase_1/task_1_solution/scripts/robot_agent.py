import rospy

from robot_sensor import UcvRobotSensor
from robot_control import UcvRobotControl


class UcvRobotAgent:
    def __init__(self, node_id = 'ucv_task_1_solver_robot_agent'):
        self._node_id = node_id

        rospy.loginfo(f'Starting Task 1 Solver Agent {self._node_id!r}')
        rospy.init_node(node_id, anonymous = False)

        self.sensor = UcvRobotSensor(debug = True)
        self.control = UcvRobotControl(self.sensor)

    def run(self):
        rospy.spin()
        rospy.loginfo(f'Terminating Task 1 Solver Agent {self._node_id!r}')
