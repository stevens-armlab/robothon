#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from ur5_client.ur5 import ur5

desired_twist = np.array([0, 0, 0, 0, 0, 0])
jog_linear_speed = 0.2  # m/s
jog_angular_speed = 0.5 # rad/s

def joy_cb(data):
    x_vel = -data.axes[0]*jog_linear_speed
    y_vel = data.axes[1]*jog_linear_speed
    z_vel = data.axes[4]*jog_linear_speed
    angular_vel_x = -data.axes[6]*jog_angular_speed/2.0
    angular_vel_y = data.axes[7]*jog_angular_speed/2.0
    angular_vel_z = -data.axes[3]*jog_angular_speed
    global desired_twist
    desired_twist = np.array([x_vel, y_vel, z_vel, angular_vel_x , angular_vel_y, angular_vel_z])

if __name__ == '__main__':
    rospy.init_node('ur5_teleop', anonymous=True)
    robot = ur5()
    rospy.Subscriber("/joy", Joy, joy_cb)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        robot.jog(desired_twist)
        rate.sleep()
