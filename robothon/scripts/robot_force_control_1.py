#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from robot_client.robot import robot
desired_twist = np.array([0, 0, 0, 0, 0, 0])
jog_linear_speed = 0.2  # m/s
jog_angular_speed = 0.5  # rad/s
enable = False

def joy_cb(data):
    global enable
    enable = data.buttons[0]

    x_vel = -data.axes[0] * jog_linear_speed
    y_vel = data.axes[1] * jog_linear_speed
    z_vel = data.axes[4] * jog_linear_speed
    angular_vel_x = -data.axes[6] * jog_angular_speed / 2.0
    angular_vel_y = data.axes[7] * jog_angular_speed / 2.0
    angular_vel_z = -data.axes[3] * jog_angular_speed
    global desired_twist
    desired_twist = np.array([x_vel, y_vel, z_vel, angular_vel_x, angular_vel_y, angular_vel_z])

    global robot1
    if data.buttons[5]:
        robot1.hand.grasp(force=30)
    elif data.buttons[4]:
        robot1.hand.ungrasp()
    else:
        robot1.hand.stop_grasp()

    if data.buttons[2]:
        robot1.hand.spread()
    elif data.buttons[3]:
        robot1.hand.unspread()

if __name__ == '__main__':
    rospy.init_node('robot_force_control', anonymous=True)
    robot1 = robot()
    rospy.Subscriber("/joy", Joy, joy_cb)
    rate = rospy.Rate(25)
    while not rospy.is_shutdown():
        if enable:
            robot1.force_control(v_des=desired_twist[0:3], fz=-10)
        else:
            robot1.arm.jog(desired_twist)
        rate.sleep()
