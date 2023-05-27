import rospy

from robot_perception import UcvRobotPerception
from robot_control import UcvRobotControl
from robot_planner import UcvRobotPlanner


class UcvRobotAgent:
    def __init__(self, node_id = 'ucv_task_1_solver_robot_agent', debug = False):
        self._node_id = node_id
        self.debug = debug

        self._log(f'Starting Task 1 Solver Agent {self._node_id!r}')
        rospy.init_node(node_id, anonymous=False)

        self.perception = UcvRobotPerception(
            debug=False
        )
        self.control = UcvRobotControl(
            publishing_rate_in_hz=10,
            debug=self.debug
        )
        self.planner = UcvRobotPlanner(
            perception=self.perception,
            control=self.control,
            debug=debug
        )

    def _log(self, message):
        if self.debug is True:
            rospy.loginfo(message)

    def _on_shutdown():
        self.control.stop()

    def run(self):
        """Continuously execute the agent to achieve the goal."""
        self._log(f'-- Initializing {self._node_id!r} node...')
        rospy.on_shutdown(self._on_shutdown)

        try:
            while not rospy.is_shutdown():
                self.execute()
        except rospy.exceptions.ROSInterruptException:
            self._log('-- Received interrupt signal')

        self._log(f'-- Terminating {self._node_id!r} node.')

    def execute(self):
        """Execute the plan using control mechanisms to achieve the goal."""
        if (plan := self.planner.plan()) is not None:
            self.control.execute_plan(plan)
