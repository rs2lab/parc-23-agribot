import rospy

from robot_perception import UcvRobotPerception
from robot_control import UcvRobotControl
from robot_planner import UcvRobotPlanner


class UcvRobotAgent:
    def __init__(self, node_id = 'ucv_task_1_solver_robot_agent', debug = False):
        self._node_id = node_id
        self.debug = debug

        rospy.init_node(node_id, anonymous=False, log_level=rospy.DEBUG if debug else rospy.INFO)
        rospy.on_shutdown(self._on_shutdown)
        rospy.logdebug(f'Starting Task 1 Solver Agent {self._node_id!r}')

        self.perception = UcvRobotPerception()

        self.control = UcvRobotControl(
            rate=10,
            latch=True,
            queue_size=0,
        )

        self.planner = UcvRobotPlanner(
            perception=self.perception,
            control=self.control,
        )

    def _on_shutdown(self):
        self.control.stop()

    def run(self):
        """Continuously execute the agent to achieve the goal."""
        rospy.loginfo(f'-- Initializing {self._node_id!r} node...')

        try:
            while not rospy.is_shutdown():
                self.execute()
        except rospy.exceptions.ROSInterruptException:
            rospy.loginfo('-- Received interrupt signal')

        rospy.loginfo(f'-- Terminating {self._node_id!r} node.')

    def execute(self):
        """Execute the plan using control mechanisms to achieve the goal."""
        if (plan := self.planner.plan()) is not None:
            self.control.execute_plan(plan)
