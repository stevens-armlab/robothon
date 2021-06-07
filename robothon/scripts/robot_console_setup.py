#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from robot_client.robot import robot
import numpy as np

if __name__ == '__main__':
    rospy.init_node('robot_console', anonymous=True)
    robot1 = robot()

