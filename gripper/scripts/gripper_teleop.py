#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from gripper_client.driver import gripper_driver

grip = gripper_driver()


def joy_cb(data):
    global hand
    if data.buttons[5]:
        grip.grasp()
    elif data.buttons[4]:
        grip.ungrasp()
    else:
        grip.stop_grasp()

if __name__ == '__main__':
    rospy.init_node('gripper_teleop', anonymous=True)
    rospy.Subscriber("/joy", Joy, joy_cb)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()