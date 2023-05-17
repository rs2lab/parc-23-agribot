#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image


def callback(data: Image):
    """This is called when the publisher publishes a new image to
    the topic '/camera/image_raw' receiving the new published value as
    the variable data of type Image. To see the specification of the type
    Image run on the terminal the following command:
    ```
    rosmsg info sensor_msgs/Image
    ```
    """
    rospy.loginfo(data.data)


if __name__ == '__main__':
    rospy.init_node('test_camera_listener')
    rospy.Subscriber('/camera/image_raw', Image, callback)
    rospy.spin()


