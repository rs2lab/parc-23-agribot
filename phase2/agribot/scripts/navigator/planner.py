import rospy

from utils.action import SteppedAction
from perceptor import AgribotPerceptor


class AgribotPlanner:
    def __init__(self, perception: AgribotPerceptor) -> None:
        self._perception = perception

    def plan_action(self) -> None:
        pass # TODO


