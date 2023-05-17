#!/usr/bin/env python3
import rospy

# TODO: add more code here

if __name__ == '__main__':
    rospy.init_node('task_1_solver')
    rospy.loginfo('Running Task 1 Solver')
    rospy.loginfo('Hit \'Ctrl + c\' to quit, or run \'rosnode kill /task_1_solver\'')
    rospy.spin()

