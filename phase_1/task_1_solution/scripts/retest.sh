#!/bin/bash

rosservice call /gazebo/reset_world

./main.py
