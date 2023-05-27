# Info

This was created while following [this tutorial](https://parc-robotics.github.io/documentation-2023/getting-started-tutorials/getting-started-with-ros/#writing-your-first-ros-package), using the follwoing command:

```bash
catkin_create_pkg test_publisher roscpp rospy std_msgs geometry_msgs
```

### Executing

First launch one of the tasks, by example, the **task1**:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch parc_robot task1.launch
```

Then, to execute the python example run the following commands:

```
cd catkin_ws
catkin_make
chmod +x ~/catkin_ws/src/parc-unicv/test_publisher/scripts/robot_publisher.py
rosrun test_publisher robot_publisher.py
```
