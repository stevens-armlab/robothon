#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from robot_client.robot import robot
import numpy as np
import rospy
from onrobot_hex_client.driver import onrobot_hex
from ur5_client.ur5 import ur5
from hand_client.driver import hand_driver
from vision_client.vision import d435i
import numpy as np
import geometry_msgs
import tf2_ros, tf
from tf import TransformListener
import open3d as o3d
import os

if __name__ == '__main__':
    rospy.init_node('robot_console', anonymous=True)
    robot1 = robot()
    tf1 = TransformListener()
    task_board_T_button_trans = [0.079, -0.147, 0.163]
    task_board_T_button_rot = [0.597, 0.523, 0.503, -0.343]
    task_board_T_button = tf.transformations.quaternion_matrix(task_board_T_button_rot)
    task_board_T_button[0:3, 3] = task_board_T_button_trans

    robot1.arm.traj_controller_enabled = False
    robot1.arm.enable_traj_controller()

    t, source, target = robot1.cam.get_transform()
    target = target.transform(t)
    target.paint_uniform_color((1, 0, 0))
    source.paint_uniform_color((0, 0, 1))
    now = rospy.Time.now()
    tf1.waitForTransform("base_link", "camera_depth_optical_frame", now, rospy.Duration(4.0))
    (base_T_cam_trans, base_T_cam_rot) = tf1.lookupTransform("base_link", "camera_depth_optical_frame", now)
    base_T_cam = tf.transformations.quaternion_matrix(base_T_cam_rot)
    base_T_cam[0:3, 3] = base_T_cam_trans

    br = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_link"
    static_transformStamped.child_frame_id = "task_board"

    mat = np.dot(base_T_cam, np.linalg.inv(t))

    trans = tf.transformations.translation_from_matrix(mat)
    quat = tf.transformations.quaternion_from_matrix(mat)
    static_transformStamped.transform.translation.x = trans[0]
    static_transformStamped.transform.translation.y = trans[1]
    static_transformStamped.transform.translation.z = trans[2]
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
    br.sendTransform(static_transformStamped)

    now = rospy.Time.now()
    tf1.waitForTransform("world", "task_board", now, rospy.Duration(4.0))
    (base_T_task_trans, base_T_task_rot) = tf1.lookupTransform("world", "task_board", now)
    base_T_task = tf.transformations.quaternion_matrix(base_T_task_rot)
    base_T_task[0:3, 3] = base_T_task_trans
    pose_goal_mat = np.dot(base_T_task, task_board_T_button)
    pose_goal_rot = tf.transformations.quaternion_from_matrix(pose_goal_mat)
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = pose_goal_rot[0]
    pose_goal.orientation.y = pose_goal_rot[1]
    pose_goal.orientation.z = pose_goal_rot[2]
    pose_goal.orientation.w = pose_goal_rot[3]
    pose_goal.position.x = pose_goal_mat[0, 3]
    pose_goal.position.y = pose_goal_mat[1, 3]
    pose_goal.position.z = pose_goal_mat[2, 3]
    plan, fraction = robot1.arm.move_group.compute_cartesian_path([pose_goal], 0.01, 0.0)
    plan = robot1.arm.move_group.retime_trajectory(robot1.arm.move_group.get_current_state(), plan,
                                                   velocity_scaling_factor=0.15, acceleration_scaling_factor=0.1)
    robot1.arm.display_trajectory(plan)
    robot1.arm.move_group.execute(plan)
    robot1.press_button()
    robot1.arm.enable_traj_controller()



