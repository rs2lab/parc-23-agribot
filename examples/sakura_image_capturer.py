#!/usr/bin/python3

import os
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


TOPIC='/front/zed_nodelet_front/rgb_raw/image_raw_color'


try:
    TOPIC=os.getenv('TOPIC', TOPIC)
except:
    pass


counter=0

def imgmsg_to_cv2(imgmsg):
    return CvBridge().imgmsg_to_cv2(imgmsg)

def handler(data):
    global counter
    image = imgmsg_to_cv2(data)
    counter += 1
    cv2.imwrite(f'{counter}.png', image)

rospy.init_node('sakura_recorder', anonymous=True)
rospy.Subscriber(TOPIC, Image, handler)

while not rospy.is_shutdown():
    print('executing, counter = %d' % counter)

