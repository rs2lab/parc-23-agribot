#!/usr/bin/env python3
import os

from robot_agent import UcvRobotAgent

DEBUG = os.getenv('DEBUG', default=False)

if __name__ == '__main__':
    agent = UcvRobotAgent(debug=DEBUG)
    agent.run()
