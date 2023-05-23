import rospy

from robot_perception import UcvRobotPerception
from robot_control import UcvRobotControl
from robot_planner import UcvRobotPlanner


class UcvRobotAgent:
    def __init__(self, node_id = 'ucv_task_1_solver_robot_agent', debug = False):
        self._node_id = node_id

        rospy.loginfo(f'Starting Task 1 Solver Agent {self._node_id!r}')
        rospy.init_node(node_id, anonymous=False)

        self.perception = UcvRobotPerception(debug=debug)
        self.control = UcvRobotControl(publishing_rate_in_hz=10)
        self.planner = UcvRobotPlanner(control=self.control, perception=self.perception)

    def run(self):
        rospy.loginfo(f'-- Initializing {self._node_id!r} node...')

        try:
            while rospy.is_shutdown() is False:
                self.planner.execute()
        except rospy.exceptions.ROSInterruptException:
            rospy.loginfo('-- Received interrupt signal')

        rospy.loginfo(f'-- Terminating {self._node_id!r} node.')
