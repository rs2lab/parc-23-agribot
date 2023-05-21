#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix


def callback(data: NavSatFix):
    """This is called when the publisher publishes a new gps info to
    the topic '/gps/fix' receiving the new published value as
    the variable data of type NavSatFix. To see the specification of the type
    NavSatFix run on the terminal the following command:
    ```
    rosmsg info sensor_msgs/NavSatFix
    ```
    """
    rospy.loginfo(f'LATI = {data.latitude}, LONGI = {data.longitude}, ALTI = {data.altitude}')


if __name__ == '__main__':
    rospy.init_node('task_gps_listener')
    rospy.Subscriber('/gps/fix', NavSatFix, callback)
    rospy.spin()


