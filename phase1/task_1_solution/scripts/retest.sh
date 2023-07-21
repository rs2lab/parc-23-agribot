#!/bin/bash

rosservice call /gazebo/reset_world && ./main.py

if [[ $? -ne 0 ]]
then
    echo "Error: Unsucessful execution of the agent!"
fi
