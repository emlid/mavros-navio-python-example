#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "\nlinear acceleration:\nx: [{}]\ny: [{}]\nz: [{}]".
    format(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/mavros/imu/data", Imu, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()

