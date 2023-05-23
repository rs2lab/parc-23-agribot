import rospy

from robot_perception import UcvRobotPerception
from robot_control import UcvRobotControl


class UcvRobotAgent:
    def __init__(self, node_id = 'ucv_task_1_solver_robot_agent', debug = False):
        self._node_id = node_id

        rospy.loginfo(f'Starting Task 1 Solver Agent {self._node_id!r}')
        rospy.init_node(node_id, anonymous=False)

        self.perception = UcvRobotPerception(debug=debug)
        self.control = UcvRobotControl(self.perception)

    def run(self):
        rospy.spin()
        rospy.loginfo(f'Terminating Task 1 Solver Agent {self._node_id!r}')
