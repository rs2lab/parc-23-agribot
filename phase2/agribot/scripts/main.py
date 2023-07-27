import os

from navigator.agent import AgribotAgent
from navigator.utils.constants import NODE_DEFAULT_NAME


debug = False

try:
    debug = bool(os.getenv('DEBUG', debug))
except:
    pass


if __name__ == '__main__':
    # To see debug info run this script with:
    # ```DEBUG=1 ./main.py``` or ```DEBUG=1 python3 main.py```
    agent = AgribotAgent(debug=debug, name_id=NODE_DEFAULT_NAME)
    agent.run()
