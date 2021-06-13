#!/usr/bin/env python
import rospy
import numpy as np
from robot_client.robot import robot
import tf2_ros, tf
from tf import TransformListener
import csv
import open3d as o3d

if __name__ == '__main__':
    rospy.init_node('robot_hand_eye_calib', anonymous=True)
    robot1 = robot()
    tf1 = TransformListener()

    user_input = raw_input("Take a capture?")
    i = 1
    with open('/home/kevin/tf_poses_timestamped.csv', mode='w') as ee_file, \
            open('/home/kevin/camera_poses_timestamped.csv', mode='w') as cam_file:
        ee_writer = csv.writer(ee_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        cam_writer = csv.writer(cam_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        while user_input == '':

            now = rospy.Time.now()
            tf1.waitForTransform("base_link", "ee_link", now, rospy.Duration(4.0))
            (base_T_ee_trans, base_T_ee_rot) = tf1.lookupTransform("base_link", "ee_link", now)

            t, source, target = robot1.cam.get_transform()
            obj_pose_trans = tf.transformations.translation_from_matrix(t).tolist()
            obj_pose_rot = tf.transformations.quaternion_from_matrix(t).tolist()

            target = target.transform(t)
            target.paint_uniform_color((1, 0, 0))
            source.paint_uniform_color((0, 0, 1))
            o3d.visualization.draw_geometries([source, target])

            user_input = raw_input("Would you like to save this scan")
            if user_input == '':
                ee_writer.writerow([i] + base_T_ee_trans + base_T_ee_rot)
                cam_writer.writerow([i] + obj_pose_trans + obj_pose_rot)
                print("Saved Scan #%d" % i)
                i += 1

            user_input = raw_input("Take another capture?")
        print("Program Ending")






