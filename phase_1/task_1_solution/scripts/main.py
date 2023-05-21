#!/usr/bin/env python3
from robot_agent import UcvRobotAgent


if __name__ == '__main__':
    rospy.loginfo('Starting Task 1 Solver...')
    agent = UcvRobotAgent()
    agent.run()
    rospy.loginfo('Finishing Task 1 Solver...')
