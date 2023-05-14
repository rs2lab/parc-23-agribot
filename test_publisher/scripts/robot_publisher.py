#!/usr/bin/env python3
from geometry_msgs.msg import Twist

import rospy
import time

def move_robot():
    rospy.init_node('robot_publisher', anonymous=True)

    # create a publisher which can talk to Robot and tell it to move
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # set publish rate at 10 Hz
    rate = rospy.Rate(10)

    # create a Twist message and add linear x and angular z values
    move_cmd = Twist()

    # --- move straight ---
    print('Moving straight')
    # move in X axis at 0.3 m/s
    move_cmd.linear.x = 0.3
    move_cmd.angular.z = 0.0

    now = time.time()
    # for the next 3 seconds publish cmd_vel move commands
    while time.time() - now < 3:
        pub.publish(move_cmd)
        rate.sleep()

    # --- rotating counterclockwise ---
    print('Rotating')
    move_cmd.linear.x = 0.0
    # rotate at 0.2 rad/sec
    move_cmd.angular.z = 0.2

    now = time.time()
    # for the next 3 seconds publish cmd_vel move commands
    while time.time() - now < 3:
        pub.publish(move_cmd)
        rate.sleep()

    # --- stopping ---
    print('Stopping')
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0

    now = time.time()
    while time.time() - now < 1:
        pub.publish(move_cmd)
        rate.sleep()

    print('Exit')

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass


