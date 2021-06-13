import rospy

from onrobot_hex_client.driver import onrobot_hex
from ur5_client.ur5 import ur5
from gripper_client.driver import gripper_driver
from vision_client.vision import d435i
import numpy as np
import geometry_msgs
import tf2_ros, tf
from tf import TransformListener
import open3d as o3d
import os
import transforms

class robot:
    def __init__(self):
        self.arm = ur5()
        self.force_sensor = onrobot_hex()
        self.gripper = gripper_driver()
        self.cam = d435i()

        self.p_c_b = None
        self.p_obj_b = None
        self.scan_index = None

        self.tf1 = TransformListener()

        self.source_pcl = None
        self.target_pcl = None
        self.t = None

        # br = tf2_ros.StaticTransformBroadcaster()
        # static_transformStamped = geometry_msgs.msg.TransformStamped()
        # static_transformStamped.header.stamp = rospy.Time.now()
        # static_transformStamped.header.frame_id = "ee_link"
        # static_transformStamped.child_frame_id = "camera_link"
        # trans = tf.transformations.translation_from_matrix(transforms.ee_T_camera_link)
        # quat = tf.transformations.quaternion_from_matrix(transforms.ee_T_camera_link)
        # static_transformStamped.transform.translation.x = trans[0]
        # static_transformStamped.transform.translation.y = trans[1]
        # static_transformStamped.transform.translation.z = trans[2]
        # static_transformStamped.transform.rotation.x = quat[0]
        # static_transformStamped.transform.rotation.y = quat[1]
        # static_transformStamped.transform.rotation.z = quat[2]
        # static_transformStamped.transform.rotation.w = quat[3]
        # br.sendTransform(static_transformStamped)

    def press_button(self):
        rate = rospy.Rate(10)
        self.gripper.go_to_closed_pos()
        rospy.timer.sleep(.5)
        time_start = rospy.get_time()
        self.force_sensor.zero()
        while self.force_sensor.get_fz() > -2:
            self.arm.jog(np.array([0, 0, -0.03, 0, 0, 0]))
            if (rospy.get_time() - time_start) > 20.0:
                print("Timed out before button was found. ")
                self.arm.jog(np.array([0, 0, 0, 0, 0, 0]))
                return
            rate.sleep()

        time_start = rospy.get_time()
        while (rospy.get_time() - time_start) < 2:
            self.arm.jog(np.array([0, 0, 0.03, 0, 0, 0]))
            rate.sleep()

        self.arm.jog(np.array([0, 0, 0, 0, 0, 0]))

    def insert_key_better(self, start_pose, slot_z_thresh=0.342):
        grid_points = np.loadtxt('key_probe_points.txt')
        stepover = 0.0005
        self.force_sensor.zero()
        rate = rospy.Rate(25)
        slot_found = False
        k = 0.00025
        for point in grid_points:
            task_board_T_key_grid = start_pose.copy()
            task_board_T_key_grid[0:2, 3] += point*stepover
            self.execute_plan(self.plan_tb_goals([task_board_T_key_grid]))

            while self.force_sensor.get_fz() > -7.5 and not slot_found:
                f_error = -15 - self.force_sensor.get_fz()
                v = k * f_error
                self.arm.jog([0, 0, v, 0, 0, 0])
                if self.get_eef_pose_mat()[2,3] < slot_z_thresh:
                    slot_found = True
                rate.sleep()
            self.arm.jog(0)
            if slot_found:
                break
            self.execute_plan(self.plan_tb_goals([task_board_T_key_grid]))
        if slot_found:
            while self.force_sensor.get_fz() > -15:
                self.arm.jog(np.array([0, 0, -0.005, 0, 0, 0]))
            self.arm.jog(0)
            self.gripper.ungrasp()
            rospy.sleep(.25)
            time_start = rospy.get_time()
            while (rospy.get_time() - time_start) < .25:
                self.arm.jog(np.array([0, 0, 0.03, 0, 0, 0]))
                rate.sleep()

            self.arm.jog(np.array([0, 0, 0, 0, 0, 0]))

    def insert_plug(self, start_pose, slot_z_thresh):
        grid_points = np.loadtxt('key_probe_points.txt')
        stepover = 0.0005
        self.force_sensor.zero()
        rate = rospy.Rate(25)
        slot_found = False
        k = 0.00025

        now = rospy.Time.now()
        self.tf1.waitForTransform("tool0", "world", now, rospy.Duration(4.0))
        tool_T_world_trans, tool_T_world_rot = self.tf1.lookupTransform("tool0", "world", now)
        tool_T_world = tf.transformations.quaternion_matrix(tool_T_world_rot)
        base_z_in_sensor_frame = tool_T_world[0:3, 2]

        def get_force():
            return np.dot(base_z_in_sensor_frame, self.force_sensor.get_force())

        for point in grid_points:
            task_board_T_key_grid = start_pose.copy()
            task_board_T_key_grid[0:2, 3] += point * stepover
            self.execute_plan(self.plan_tb_goals([task_board_T_key_grid]))

            while get_force() < 5 and not slot_found:
                f_error = 15 - get_force()
                v = -k * f_error
                self.arm.jog([0, 0, v, 0, 0, 0])
                if self.get_eef_pose_mat()[2, 3] < slot_z_thresh:
                    slot_found = True
                rate.sleep()
            self.arm.jog(0)
            if slot_found:
                break
            self.execute_plan(self.plan_tb_goals([task_board_T_key_grid]))
        if slot_found:
            while get_force() < 10:
                self.arm.jog(np.array([0, 0, -0.005, 0, 0, 0]))
            self.arm.jog(0)
            self.gripper.ungrasp()
            rospy.sleep(.25)
            time_start = rospy.get_time()
            while (rospy.get_time() - time_start) < .5:
                self.arm.jog(np.array([0, 0, 0.03, 0, 0, 0]))
                rate.sleep()

            self.arm.jog(np.array([0, 0, 0, 0, 0, 0]))

            self.self_centering_grasp(6, force=40)
            while get_force() < 15:
                self.arm.jog(np.array([0, 0, -0.005, 0, 0, 0]))
            self.arm.jog(0)
            self.gripper.ungrasp()
            rospy.sleep(.25)
            time_start = rospy.get_time()
            while (rospy.get_time() - time_start) < 4:
                self.arm.jog(np.array([0, 0, 0.03, 0, 0, 0]))
                rate.sleep()

            self.arm.jog(np.array([0, 0, 0, 0, 0, 0]))

    def force_control(self, k=0.00025, fz=-10, v_des=np.array([0.005, 0, 0])):
        n = np.array([0, 0, 1])
        force_spec_matrix = np.outer(n, n)
        f_des = np.array([0, 0, fz])
        f_error = f_des - self.force_sensor.get_force() # Negative fz -> tool pushing on surface
        vf = k * np.dot(force_spec_matrix, f_error)
        v = np.dot((np.eye(3) - force_spec_matrix), v_des) + vf
        
        desired_twist = np.zeros(6)
        desired_twist[0:3] = v

        self.arm.jog(desired_twist)

    def plan_tb_goals(self, task_board_T_goal_list):
        now = rospy.Time.now()
        self.tf1.waitForTransform("world", "task_board", now, rospy.Duration(4.0))
        (base_T_task_trans, base_T_task_rot) = self.tf1.lookupTransform("world", "task_board", now)
        base_T_task = tf.transformations.quaternion_matrix(base_T_task_rot)
        base_T_task[0:3, 3] = base_T_task_trans

        pose_goal_list = []
        for task_board_T_goal in task_board_T_goal_list:
            pose_goal_mat = np.dot(base_T_task, task_board_T_goal)
            pose_goal_rot = tf.transformations.quaternion_from_matrix(pose_goal_mat)
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.orientation.x = pose_goal_rot[0]
            pose_goal.orientation.y = pose_goal_rot[1]
            pose_goal.orientation.z = pose_goal_rot[2]
            pose_goal.orientation.w = pose_goal_rot[3]
            pose_goal.position.x = pose_goal_mat[0, 3]
            pose_goal.position.y = pose_goal_mat[1, 3]
            pose_goal.position.z = pose_goal_mat[2, 3]
            pose_goal_list.append(pose_goal)
        plan, fraction = self.arm.move_group.compute_cartesian_path(pose_goal_list, 0.01, 0.0)
        plan = self.arm.move_group.retime_trajectory(self.arm.move_group.get_current_state(), plan,
                                                       velocity_scaling_factor=0.15, acceleration_scaling_factor=0.1)
        self.arm.display_trajectory(plan)

        return plan

    def compute_task_board_registration(self):
        now = rospy.Time.now()
        self.tf1.waitForTransform("world", "camera_depth_optical_frame", now, rospy.Duration(4.0))
        (base_T_cam_trans, base_T_cam_rot) = self.tf1.lookupTransform("world", "camera_depth_optical_frame", now)
        base_T_cam = tf.transformations.quaternion_matrix(base_T_cam_rot)
        base_T_cam[0:3, 3] = base_T_cam_trans

        self.t, self.source_pcl, self.target_pcl = self.cam.get_transform()

        self.target_pcl = self.target_pcl.transform(self.t)
        self.target_pcl.paint_uniform_color((1, 0, 0))
        self.source_pcl.paint_uniform_color((0, 0, 1))

        br = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "base_link"
        static_transformStamped.child_frame_id = "task_board"

        mat = np.dot(base_T_cam, np.linalg.inv(self.t))

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

    def execute_plan(self, plan):
        self.arm.enable_traj_controller()
        self.arm.move_group.execute(plan)

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

        br = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "base_link"
        static_transformStamped.child_frame_id = "task_board_center"
        static_transformStamped.transform.translation.x = p_obj_b[0]
        static_transformStamped.transform.translation.y = p_obj_b[1]
        static_transformStamped.transform.translation.z = p_obj_b[2]
        static_transformStamped.transform.rotation.x = 0
        static_transformStamped.transform.rotation.y = 0
        static_transformStamped.transform.rotation.z = 0
        static_transformStamped.transform.rotation.w = 1
        br.sendTransform(static_transformStamped)

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

    def go_to_start(self):
        self.arm.enable_traj_controller()
        plan, fraction = self.arm.move_group.compute_cartesian_path([transforms.base_link_T_start], 0.01, 0.0)
        plan = self.arm.move_group.retime_trajectory(self.arm.move_group.get_current_state(), plan,
                                                       velocity_scaling_factor=0.15, acceleration_scaling_factor=0.1)
        self.arm.display_trajectory(plan)
        self.arm.move_group.execute(plan)
        self.go_to_joint_angles(transforms.start_joint_angles)

    def go_to_joint_angles(self, angles):
        self.arm.move_group.go(angles, wait=True)
        self.arm.move_group.stop()

    def self_centering_grasp(self, grasp_time, force=20):
        gripper_direction = self.get_eef_pose_mat()[0:3, 1]
        v_max = 0.02
        force_deadband = 0.05
        gain = 0.004
        rate = rospy.Rate(25)

        self.force_sensor.zero()
        rospy.sleep(.25)
        self.gripper.grasp(force=force)
        time_start = rospy.get_time()
        while (rospy.get_time() - time_start) < grasp_time:
            try:
                fx = self.force_sensor.get_fx()
                if np.abs(fx) < force_deadband:
                    self.arm.jog(0)
                else:
                    v_des = -gain*fx
                    if v_des > v_max:
                        v_des = v_max
                    elif v_des < -v_max:
                        v_des = -v_max

                    desired_twist = np.concatenate((gripper_direction * v_des, [0,0,0]))
                    self.arm.jog(desired_twist)
            except Exception, e:
                print(e)
                self.arm.jog(np.array([0, 0, 0, 0, 0, 0]))
            rate.sleep()
        self.arm.jog(np.array([0, 0, 0, 0, 0, 0]))

    def turn_key(self, angle_to_turn=32):
        rate = rospy.Rate(25)
        try:
            now = rospy.Time.now()
            self.tf1.waitForTransform('base_link', 'ee_link', now, rospy.Duration(.25))
            (base_T_ee_trans, base_T_ee_rot) = self.tf1.lookupTransform("base_link", "ee_link", now)
        except tf2_ros.TransformException:
            print('failed')
            self.robot1.arm.jog(0)
            return

        base_T_ee = tf.transformations.quaternion_matrix(base_T_ee_rot)
        ee_T_key_axis = np.array([0, 0, 0.007])
        p_ee_to_axis_in_base = base_T_ee[0:3, 0:3].dot(ee_T_key_axis)

        initial_cross = np.cross(p_ee_to_axis_in_base, np.array([0, 0, 1]))
        radius = np.linalg.norm(initial_cross)
        initial_tangential_vector = initial_cross / radius

        angle = 0
        while angle < angle_to_turn:
            try:
                now = rospy.Time.now()
                self.tf1.waitForTransform('base_link', 'ee_link', now, rospy.Duration(.25))
                (base_T_ee_trans, base_T_ee_rot) = self.tf1.lookupTransform("base_link", "ee_link", now)
            except tf2_ros.TransformException:
                print('failed')
                self.robot1.arm.jog(0)
                return

            base_T_ee = tf.transformations.quaternion_matrix(base_T_ee_rot)
            p_ee_to_axis_in_base = base_T_ee[0:3, 0:3].dot(ee_T_key_axis)

            cross = np.cross(p_ee_to_axis_in_base, np.array([0, 0, 1]))
            radius = np.linalg.norm(cross)
            tangential_vector = cross / radius

            desired_v = tangential_vector * 0.005
            desired_w = np.array([0, 0, 1]) * 0.005 / radius
            desired_twist = np.concatenate([desired_v, desired_w])
            self.arm.jog(desired_twist)

            angle = np.rad2deg(np.arccos(np.dot(initial_tangential_vector, tangential_vector)))
            rate.sleep()

        try:
            now = rospy.Time.now()
            self.tf1.waitForTransform('base_link', 'ee_link', now, rospy.Duration(.25))
            (base_T_ee_trans, base_T_ee_rot) = self.tf1.lookupTransform("base_link", "ee_link", now)
        except tf2_ros.TransformException:
            print('failed')
            self.robot1.arm.jog(0)
            return

        base_T_ee = tf.transformations.quaternion_matrix(base_T_ee_rot)
        base_T_ee[0:3, 3] = base_T_ee_trans

        ee_T_key_axis = np.array([0, 0, 0.007])

        p_ee_to_axis_in_base = base_T_ee[0:3, 0:3].dot(ee_T_key_axis)

        cross = np.cross(p_ee_to_axis_in_base, np.array([0, 0, 1]))
        radius = np.linalg.norm(cross)
        tangential_vector = cross / radius

        angle_to_turn_back = np.rad2deg(np.arccos(np.dot(initial_tangential_vector, tangential_vector)))

        initial_tangential_vector = tangential_vector

        while angle < angle_to_turn_back:
            try:
                now = rospy.Time.now()
                self.tf1.waitForTransform('base_link', 'ee_link', now, rospy.Duration(.25))
                (base_T_ee_trans, base_T_ee_rot) = self.tf1.lookupTransform("base_link", "ee_link", now)
            except tf2_ros.TransformException:
                print('failed')
                self.robot1.arm.jog(0)
                return

            base_T_ee = tf.transformations.quaternion_matrix(base_T_ee_rot)
            p_ee_to_axis_in_base = base_T_ee[0:3, 0:3].dot(ee_T_key_axis)

            cross = np.cross(p_ee_to_axis_in_base, np.array([0, 0, 1]))
            radius = np.linalg.norm(cross)
            tangential_vector = cross / radius

            desired_v = tangential_vector * -0.005
            desired_w = np.array([0, 0, 1]) * -0.005 / radius
            desired_twist = np.concatenate([desired_v, desired_w])
            self.arm.jog(desired_twist)

            angle = np.rad2deg(np.arccos(np.dot(initial_tangential_vector, tangential_vector)))
            rate.sleep()

        self.arm.jog(0)





