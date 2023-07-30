#!/usr/bin/bash


echo "Running Local: DEBUG mode is active!"


## -- Running the Agent in local mode --
DEBUG=1 ROS_IP=$(hostname -I | tr -d [:blank:]) ROS_MASTER_URI=http://localhost:11311 python3 main.py


if [[ $? -ne 0 ]]
then
    echo 'Some error occured during the execution of the agent!'
fi
