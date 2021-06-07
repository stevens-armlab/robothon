import rospy

from onrobot_hex_client.driver import onrobot_hex
from ur5_client.ur5 import ur5
from hand_client.driver import hand_driver
from vision_client.vision import d435i
import numpy as np
import geometry_msgs
import tf2_ros, tf
import open3d as o3d
import os
# ee_T_camera_depth_optical_frame = np.array([[-0., -0.1736453, 0.98480826, 0.141815],
#                                             [-0.70703996, -0.6964304, -0.12279737, 0.03879],
#                                             [0.70717359, -0.6962988, -0.12277416, 0.049377],
#                                             [0., 0., 0., 1.]])
# camera_depth_optical_frame_T_camera_link = np.array([[0, -1, 0, 0],
#                                  [0, 0, -1, 0],
#                                  [1, 0, 0, 0],
#                                  [0, 0, 0, 1]])
# ee_T_camera_link = np.array([[0.98480826, 0., 0.1736453, 0.141815],
#                              [-0.12279737, 0.70703996, 0.6964304, 0.03879],
#                              [-0.12277416, -0.70717359, 0.6962988, 0.049377],
#                              [0., 0., 0., 1.]])



class robot:
    def __init__(self):
        self.arm = ur5()
        self.force_sensor = onrobot_hex()
        self.hand = hand_driver()
        self.cam = d435i()

        self.ee_T_gripper = np.array([[0., 0., 1., 0.062],
                                      [-1., 0., 0., 0.],
                                      [0., -1., 0., 0.],
                                      [0., 0., 0., 1.]])

        self.gripper_T_camera_depth_optical_frame = np.array([[-1., 0., 0., 0.0175],
                                                              [0., -0.90630779, -0.42261826, 0.07122917],
                                                              [-0., -0.42261826, 0.90630779, 0.03935497],
                                                              [0., 0., 0., 1.]])
        self.ee_T_camera_depth_optical_frame =np.dot(self.ee_T_gripper, self.gripper_T_camera_depth_optical_frame)
        # This is for the hand, not the gripper
        # self.ee_T_camera_depth_optical_frame = np.array([[0., -0.1736453, 0.98480826, 0.13767881],
        #                                                 [-0.70703996, -0.6964304, -0.12279737, 0.05167895],
        #                                                 [0.70717359, -0.6962988, -0.12277416, 0.03751711],
        #                                                 [0., 0., 0., 1.]])
        self.camera_depth_optical_frame_T_camera_link = np.array([[0, -1, 0, 0],
                                                                  [0, 0, -1, 0],
                                                                  [1, 0, 0, 0],
                                                                  [0, 0, 0, 1]])
        self.ee_T_camera_link = np.dot(self.ee_T_camera_depth_optical_frame,
                                       self.camera_depth_optical_frame_T_camera_link)



        br = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "ee_link"
        static_transformStamped.child_frame_id = "camera_link"
        mat = self.ee_T_camera_link
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

        self.p_c_b = None
        self.p_obj_b = None
        self.scan_index = None

    def press_button(self):
        rate = rospy.Rate(10)
        self.hand.set_grasp_position(2140)
        rospy.timer.sleep(2)
        time_start = rospy.get_time()
        while self.force_sensor.get_fz() > -2:
            self.arm.jog(np.array([0, 0, -0.03, 0, 0, 0]))
            if (rospy.get_time() - time_start) > 20.0:
                print("Timed out before button was found. ")
                self.arm.jog(np.array([0, 0, 0, 0, 0, 0]))
                return
            rate.sleep()

        time_start = rospy.get_time()
        while (rospy.get_time() - time_start) < 3.0:
            self.arm.jog(np.array([0, 0, 0.03, 0, 0, 0]))
            rate.sleep()

        self.arm.jog(np.array([0, 0, 0, 0, 0, 0]))

    def force_control(self, k=0.00025, fz=-20, v_des=np.array([0.005, 0, 0])):
        n = np.array([0, 0, 1])
        force_spec_matrix = np.outer(n, n)
        f_des = np.array([0, 0, fz])
        f_error = f_des - self.force_sensor.get_force() # Negative fz -> tool pushing on surface
        vf = k * np.dot(force_spec_matrix, f_error)
        v = np.dot((np.eye(3) - force_spec_matrix), v_des) + vf
        
        desired_twist = np.zeros(6)
        desired_twist[0:3] = v

        self.arm.jog(desired_twist)

    def get_eef_pose_mat(self):
        pose = self.arm.move_group.get_current_pose().pose
        pos = pose.position
        ori = pose.orientation
        mat = tf.transformations.quaternion_matrix([ori.x, ori.y, ori.z, ori.w])
        mat[0:3, 3] = [pos.x, pos.y, pos.z]
        return mat

    def capture_and_save_pcl(self, folder_dir):
        pcl = self.cam.capture_pcl()
        o3d.io.write_point_cloud(folder_dir+'/cloud.pcd', pcl)
        cam_pose = np.dot(self.get_eef_pose_mat(), self.ee_T_camera_depth_optical_frame)
        np.savetxt(folder_dir + '/CameraPose', cam_pose)

    def rotate_cam_about_angle(self, angle):

        # position of {c} expressed in {b}
        p_c_b = self.p_c_b
        delta_p = p_c_b - self.p_obj_b
        delta_p = np.append(delta_p, 1.0)

        rot = tf.transformations.rotation_matrix(angle, (0.0, 0.0, 1.0))
        delta_p_des = np.dot(rot, delta_p)
        delta_p_des = delta_p_des[0:3]
        delta_p_des_norm = delta_p_des / np.linalg.norm(delta_p_des)
        # print rot_1.shape
        rot_cam_d = np.identity(4)
        # z axis
        rot_cam_d[0:3, 2] = -delta_p_des_norm
        # x axis
        rot_cam_d[0:3, 0] = np.cross(rot_cam_d[0:3, 2], np.array((0, 0, 1)))
        rot_cam_d[0:3, 0] = rot_cam_d[0:3, 0] / np.linalg.norm(rot_cam_d[0:3, 0])
        # y axis
        rot_cam_d[0:3, 1] = np.cross(rot_cam_d[0:3, 2], rot_cam_d[0:3, 0])
        rot_cam_d[0:3, 1] = rot_cam_d[0:3, 1] / np.linalg.norm((rot_cam_d[0:3, 1]))

        b_T_cam_d = rot_cam_d.copy()
        b_T_cam_d[0:3, 3] = self.p_obj_b + delta_p_des

        b_T_ee_d = np.dot(b_T_cam_d, np.linalg.inv(self.ee_T_camera_depth_optical_frame))

        pos_ee_d = b_T_ee_d[0:3, 3]
        rot_ee_d = tf.transformations.quaternion_from_matrix(b_T_ee_d)

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = rot_ee_d[0]
        pose_goal.orientation.y = rot_ee_d[1]
        pose_goal.orientation.z = rot_ee_d[2]
        pose_goal.orientation.w = rot_ee_d[3]
        pose_goal.position.x = pos_ee_d[0]
        pose_goal.position.y = pos_ee_d[1]
        pose_goal.position.z = pos_ee_d[2]
        plan, fraction = self.arm.move_group.compute_cartesian_path([pose_goal], 0.01, 0.0)
        plan = self.arm.move_group.retime_trajectory(self.arm.move_group.get_current_state(), plan,
                                                     velocity_scaling_factor=0.15, acceleration_scaling_factor=0.1)
        self.arm.display_trajectory(plan)
        print("Fraction: %f" % fraction)
        inp = raw_input("Execute the plan or not?")
        if inp != "":
            exit()
        self.arm.move_group.execute(plan, wait=True)

        inp = raw_input("Ready to take capture?")
        if inp != "":
            exit()
        folder = '/home/kevin/Scan%.4d' % self.scan_index
        os.mkdir(folder)
        self.capture_and_save_pcl(folder)
        self.scan_index += 1

    def rotate_multiple_times(self, n, p_obj_b=np.array([0.02, 0.43, 0.10])):
        self.arm.enable_traj_controller()
        # position of object expressed in {b}
        self.p_obj_b = p_obj_b

        # Start position of camera in base frame
        p_c_b = np.array((0, 0.347, 0.475))
        delta_p = p_c_b - p_obj_b
        delta_p = np.append(delta_p, 1.0)

        delta_p_des = delta_p
        delta_p_des = delta_p_des[0:3]
        delta_p_des_norm = delta_p_des / np.linalg.norm(delta_p_des)

        rot_cam_d = np.identity(4)
        # Desired camera frame z axis (expressed in base frame)
        rot_cam_d[0:3, 2] = -delta_p_des_norm
        # x axis
        rot_cam_d[0:3, 0] = np.cross(rot_cam_d[0:3, 2], np.array((0, 0, 1)))
        rot_cam_d[0:3, 0] = rot_cam_d[0:3, 0] / np.linalg.norm(rot_cam_d[0:3, 0])
        # y axis
        rot_cam_d[0:3, 1] = np.cross(rot_cam_d[0:3, 2], rot_cam_d[0:3, 0])
        rot_cam_d[0:3, 1] = rot_cam_d[0:3, 1] / np.linalg.norm((rot_cam_d[0:3, 1]))

        b_T_cam_d = rot_cam_d.copy()
        b_T_cam_d[0:3, 3] = p_c_b

        b_T_ee_d = np.dot(b_T_cam_d, np.linalg.inv(self.ee_T_camera_depth_optical_frame))

        pos_ee_d = b_T_ee_d[0:3, 3]
        rot_ee_d = tf.transformations.quaternion_from_matrix(b_T_ee_d)

        self.p_c_b = p_c_b

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = rot_ee_d[0]
        pose_goal.orientation.y = rot_ee_d[1]
        pose_goal.orientation.z = rot_ee_d[2]
        pose_goal.orientation.w = rot_ee_d[3]
        pose_goal.position.x = pos_ee_d[0]
        pose_goal.position.y = pos_ee_d[1]
        pose_goal.position.z = pos_ee_d[2]

        # self.move_group.set_pose_target(pose_goal)
        # plan = self.move_group.plan()

        plan, fraction = self.arm.move_group.compute_cartesian_path([pose_goal], 0.01, 0.0)
        plan = self.arm.move_group.retime_trajectory(self.arm.move_group.get_current_state(), plan,
                                                     velocity_scaling_factor=0.15, acceleration_scaling_factor=0.1)
        self.arm.display_trajectory(plan)
        print("Fraction: %f" % fraction)
        inp = raw_input("Execute the plan or not?")
        if inp != "":
            exit()
        self.arm.move_group.execute(plan, wait=True)

        inp = raw_input("Ready to take capture?")
        if inp != "":
            exit()
        self.scan_index = 0
        folder = '/home/kevin/Scan%.4d' % self.scan_index
        os.mkdir(folder)
        self.capture_and_save_pcl(folder)
        self.scan_index += 1

        for i in range(1, n):
            # raw_input()
            angle = 2 * i * np.pi / n
            self.rotate_cam_about_angle(angle)


