#!/usr/bin/env python3
import os

from robot_agent import UcvRobotAgent

DEBUG = False

try:
    DEBUG = bool(os.getenv('DEBUG', DEBUG))
except:
    pass

if __name__ == '__main__':
    # To see debug info run this script with:
    # ```DEBUG=1 ./main.py``` or ```DEBUG=1 python3 main.py``` 
    agent = UcvRobotAgent(debug=DEBUG)
    agent.run()
