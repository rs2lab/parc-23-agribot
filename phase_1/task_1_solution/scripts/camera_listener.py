#!/usr/bin/env python3
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import segmentation as s


def callback(data: Image):
    """This is called when the publisher publishes a new image to
    the topic '/camera/image_raw' receiving the new published value as
    the variable data of type Image. To see the specification of the type
    Image run on the terminal the following command:
    ```
    rosmsg info sensor_msgs/Image
    ```
    """
    br = CvBridge()
    rospy.loginfo('Receiving video frame')
    frame = br.imgmsg_to_cv2(data)
    cv2.imshow('camera', s.detect_ground(frame))
    cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('test_camera_listener')
    rospy.Subscriber('/camera/image_raw', Image, callback)
    rospy.spin()
    cv2.destroyAllWindows()

