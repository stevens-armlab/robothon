#!/usr/bin/env python
import rospy
from robot_client.robot import robot
import numpy as np
import rospy
import numpy as np
import geometry_msgs
import tf, tf2_ros
from tf import TransformListener
import robot_client.transforms as transforms
from sensor_msgs.msg import Joy

desired_twist = np.array([0, 0, 0, 0, 0, 0])
jog_linear_speed = 0.1  # m/s
joy_x = 0
def joy_cb(data):
    global joy_x
    joy_x = -(data.axes[0] ** 3)

if __name__ == '__main__':
    rospy.init_node('rotate_about_cam_axes', anonymous=True)
    robot1 = robot()
    rospy.Subscriber("/joy", Joy, joy_cb)
    rate = rospy.Rate(100)
    tf1 = TransformListener()
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            tf1.waitForTransform('base_link', 'ee_link', now, rospy.Duration(.25))
            (base_T_ee_trans, base_T_ee_rot) = tf1.lookupTransform("base_link", "ee_link", now)
        except tf2_ros.TransformException:
            print('failed')
            robot1.arm.jog(0)
            continue

        base_T_ee = tf.transformations.quaternion_matrix(base_T_ee_rot)
        base_T_ee[0:3, 3] = base_T_ee_trans

        base_T_cam = np.dot(base_T_ee, transforms.ee_T_camera_depth_optical_frame)

        p_cam_ee = base_T_cam[0:3, 3] - base_T_ee[0:3, 3]
        cam_z = base_T_cam[0:3, 2]

        cross = np.cross(p_cam_ee, cam_z)
        radius = np.linalg.norm(cross)

        desired_v = cross * joy_x * jog_linear_speed / radius
        desired_w = cam_z * joy_x * jog_linear_speed / radius
        desired_twist = np.concatenate([desired_v, desired_w])

        ee_T_key_axis = np.array([0, 0, 0.007])

        p_ee_to_axis_in_base = base_T_ee[0:3, 0:3].dot(ee_T_key_axis)

        cross = np.cross(p_ee_to_axis_in_base, np.array([0, 0, 1]))
        radius = np.linalg.norm(cross)

        desired_v = cross * joy_x * jog_linear_speed / radius
        desired_w = np.array([0, 0, 1]) * joy_x * jog_linear_speed / radius
        desired_twist = np.concatenate([desired_v, desired_w])

        robot1.arm.jog(desired_twist)
        rate.sleep()

