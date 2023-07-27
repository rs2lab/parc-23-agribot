import os
import navigator.utils.constants as cnst

from navigator import AgribotAgent


debug = False

try:
    debug = bool(os.getenv('DEBUG', debug))
except:
    pass


if __name__ == '__main__':
    # To see debug info run this script with:
    # ```DEBUG=1 ./main.py``` or ```DEBUG=1 python3 main.py```
    agent = AgribotAgent(name_id=cnst.NODE_DEFAULT_NAME, debug=debug)
    agent.run()
