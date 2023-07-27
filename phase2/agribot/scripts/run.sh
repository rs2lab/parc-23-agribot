#!/usr/bin/bash


echo -n "DEBUG mode is "
if [[ $DEBUG -eq 1 ]]
then
    echo "active!"
else
    echo "not active, to run the agent in DEBUG mode run it using 'DEBUG=1 ./run.sh'"
fi


## -- Running the Agent --
ROS_IP=$(hostname -I | tr -d [:blank:]) ROS_MASTER_URI='' python3 main.py


if [[ $? -ne 0 ]]
then
    echo 'Some error occured during the execution of the agent!'
fi
