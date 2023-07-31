#!/usr/bin/bash


echo -n "Running into the Robot: DEBUG mode is "
if [[ $DEBUG -eq 1 ]]
then
    echo "active!"
else
    echo "not active, to run the agent in DEBUG mode run it using 'DEBUG=1 ./run-into-robot.sh'"
fi


## -- Running the Agent into the Robot --
ROS_IP=$(hostname -I | tr -d [:blank:]) ROS_MASTER_URI=http://192.168.50.13:11311 python3 main.py


if [[ $? -ne 0 ]]
then
    echo 'Some error occured during the execution of the agent!'
fi
