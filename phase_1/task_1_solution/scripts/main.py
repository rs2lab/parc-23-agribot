#!/usr/bin/env python3
import rospy

from robot_agent import UcvRobotAgent


if __name__ == '__main__':
    agent = UcvRobotAgent()
    agent.run()
