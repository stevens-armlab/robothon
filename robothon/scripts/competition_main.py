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

if __name__ == '__main__':
    rospy.init_node('robot_console', anonymous=True)
    robot1 = robot()
    tf1 = TransformListener()

    robot1.arm.traj_controller_enabled = False
    robot1.arm.enable_traj_controller()

    rospy.sleep(.25)

    robot1.go_to_start()
    rospy.sleep(.25)
    robot1.compute_task_board_registration()
    robot1.gripper.go_to_closed_pos()

    now = rospy.Time.now()
    tf1.waitForTransform("world", "task_board", now, rospy.Duration(4.0))
    (base_T_task_trans, base_T_task_rot) = tf1.lookupTransform("world", "task_board", now)
    base_T_task = tf.transformations.quaternion_matrix(base_T_task_rot)
    base_T_task[0:3, 3] = base_T_task_trans

    tb_x_vec_in_base_link = base_T_task[0:3, 0:3].dot(transforms.task_board_T_tb_norm1)
    tb_angle = np.rad2deg(np.arctan2(tb_x_vec_in_base_link[1], tb_x_vec_in_base_link[0]))
    print(tb_angle)

    use_strategy_b = False
    if -135 < tb_angle < 15:
        use_strategy_b = False
    else:
        use_strategy_b = True
    # -180 to -90: -270 degree turn
    # [-90 to 0: -nothing]
    # [0 to 90: -rotate 90 degrees
    # Greater than 90: rotate 180 degrees
    if tb_angle < -90:
        joints_setpoint = transforms.start_joint_angles.copy()
        joints_setpoint[5] -= 1.5 * np.pi
        robot1.go_to_joint_angles(joints_setpoint)
    elif tb_angle < 0:
        joints_setpoint = transforms.start_joint_angles.copy()
        joints_setpoint[5] -= 0
    elif tb_angle < 90:
        joints_setpoint = transforms.start_joint_angles.copy()
        joints_setpoint[5] -= 0.5 * np.pi
        robot1.go_to_joint_angles(joints_setpoint)
    else:
        joints_setpoint = transforms.start_joint_angles.copy()
        joints_setpoint[5] -= np.pi
        robot1.go_to_joint_angles(joints_setpoint)

    plan = robot1.plan_tb_goals([transforms.icp_T_probe1, transforms.icp_T_probe2])
    robot1.execute_plan(plan)
    rospy.sleep(.25)

    rate = rospy.Rate(10)

    # In the below vectors, tb_norm1 is the outward facing normal vector to the right face of the task board
    # tb_norm2 is the outward facing normal vector to the back face of the task board
    v_norm_1 = base_T_task[0:3, 0:3].dot(transforms.task_board_T_tb_norm1) * -0.005
    v_norm_2 = base_T_task[0:3, 0:3].dot(transforms.task_board_T_tb_norm2) * -0.005

    v_norm_1 = np.concatenate([v_norm_1, np.zeros(3)])
    v_norm_2 = np.concatenate([v_norm_2, np.zeros(3)])

    time_start = rospy.get_time()
    robot1.force_sensor.zero()
    rospy.sleep(.25)
    while robot1.force_sensor.get_fx() > -1:
        robot1.arm.jog(v_norm_2)
        rate.sleep()
    print("Found point 1")
    robot1.arm.jog(0)

    now = rospy.Time.now()
    tf1.waitForTransform('base_link', 'ee_link', now, rospy.Duration(.25))
    (probe_point_1, _) = tf1.lookupTransform("base_link", "ee_link", now)
    probe_point_1 = np.array(probe_point_1)
    probe_point_1[2] = 0

    plan = robot1.plan_tb_goals([transforms.icp_T_probe2, transforms.icp_T_probe3])
    robot1.execute_plan(plan)
    rospy.sleep(.25)

    time_start = rospy.get_time()
    robot1.force_sensor.zero()
    while robot1.force_sensor.get_fx() > -1:
        robot1.arm.jog(v_norm_2)
        rate.sleep()
    robot1.arm.jog(0)

    now = rospy.Time.now()
    tf1.waitForTransform('base_link', 'ee_link', now, rospy.Duration(.25))
    (probe_point_2, _) = tf1.lookupTransform("base_link", "ee_link", now)
    probe_point_2 = np.array(probe_point_2)
    probe_point_2[2] = 0

    plan = robot1.plan_tb_goals([transforms.icp_T_probe3, transforms.icp_T_probe4, transforms.icp_T_probe5])
    robot1.execute_plan(plan)
    rospy.sleep(.25)

    time_start = rospy.get_time()
    robot1.force_sensor.zero()
    while robot1.force_sensor.get_fx() > -1:
        robot1.arm.jog(v_norm_1)
        rate.sleep()
    robot1.arm.jog(0)

    now = rospy.Time.now()
    tf1.waitForTransform('base_link', 'ee_link', now, rospy.Duration(.25))
    (probe_point_3, _) = tf1.lookupTransform("base_link", "ee_link", now)
    probe_point_3 = np.array(probe_point_3)
    probe_point_3[2] = 0

    plan = robot1.plan_tb_goals([transforms.icp_T_probe5])
    robot1.execute_plan(plan)

    x_vec = np.array([probe_point_2[0]-probe_point_1[0], probe_point_2[1] - probe_point_1[1], 0])
    x_vec /= np.linalg.norm(x_vec)

    z_vec = np.array([0, 0, 1])

    y_vec = np.cross(z_vec, x_vec)

    normal_dist = np.linalg.norm(np.cross(x_vec, probe_point_3 - probe_point_1))

    origin = probe_point_3 + normal_dist * y_vec

    rot_mat = np.identity(4)
    rot_mat[0:3, 0] = x_vec
    rot_mat[0:3, 1] = y_vec
    rot_mat[0:3, 2] = z_vec

    br = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_link"
    static_transformStamped.child_frame_id = "task_board"

    quat = tf.transformations.quaternion_from_matrix(rot_mat)
    static_transformStamped.transform.translation.x = origin[0]
    static_transformStamped.transform.translation.y = origin[1]
    static_transformStamped.transform.translation.z = origin[2]
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
    br.sendTransform(static_transformStamped)

    rospy.sleep(.25)

    robot1.gripper.go_to_closed_pos()
    plan = robot1.plan_tb_goals([transforms.task_board_T_pre_button, transforms.task_board_T_button])
    robot1.execute_plan(plan)

    robot1.press_button()
    robot1.gripper.ungrasp()
    rospy.sleep(.25)

    plan = robot1.plan_tb_goals([transforms.task_board_T_key1])
    robot1.execute_plan(plan)
    rospy.sleep(.25)

    plan = robot1.plan_tb_goals([transforms.task_board_T_key2])
    robot1.execute_plan(plan)
    robot1.gripper.set_grasp_position(1700)
    rospy.sleep(.25)

    time_start = rospy.get_time()
    robot1.force_sensor.zero()
    while robot1.force_sensor.get_fy() < 1:
        robot1.arm.jog(-v_norm_1)
        if (rospy.get_time() - time_start) > 20:
            print("Timed out before key was found. ")
            robot1.arm.jog(np.array([0, 0, 0, 0, 0, 0]))
            import sys
            sys.exit()
        rate.sleep()
    robot1.arm.jog(0)

    now = rospy.Time.now()
    tf1.waitForTransform("task_board", 'ee_link', now, rospy.Duration(.25))
    probe_point_4, _ = tf1.lookupTransform("task_board", 'ee_link', now)
    probe_point_4 = np.array(probe_point_4)

    x_adjustment = (probe_point_4 - transforms.probe_point_4_des)[0]

    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_link"
    static_transformStamped.child_frame_id = "task_board"

    origin = origin + x_vec * x_adjustment
    quat = tf.transformations.quaternion_from_matrix(rot_mat)
    static_transformStamped.transform.translation.x = origin[0]
    static_transformStamped.transform.translation.y = origin[1]
    static_transformStamped.transform.translation.z = origin[2]
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
    br.sendTransform(static_transformStamped)
    rospy.sleep(.25)

    plan = robot1.plan_tb_goals([transforms.task_board_T_key2])
    robot1.execute_plan(plan)
    rospy.sleep(.25)

    robot1.self_centering_grasp(5)

    now = rospy.Time.now()
    tf1.waitForTransform("task_board", 'ee_link', now, rospy.Duration(.25))
    current_pos, _ = tf1.lookupTransform("task_board", 'ee_link', now)

    error = current_pos - transforms.task_board_T_key2[0:3, 3]

    y_adjustment = error[1]

    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_link"
    static_transformStamped.child_frame_id = "task_board"

    origin = origin + y_vec * y_adjustment
    quat = tf.transformations.quaternion_from_matrix(rot_mat)
    static_transformStamped.transform.translation.x = origin[0]
    static_transformStamped.transform.translation.y = origin[1]
    static_transformStamped.transform.translation.z = origin[2]
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
    br.sendTransform(static_transformStamped)

    rospy.sleep(.25)

    plan = robot1.plan_tb_goals([transforms.task_board_T_key3])
    robot1.execute_plan(plan)
    rospy.sleep(.25)

    plan = robot1.plan_tb_goals([transforms.task_board_T_key4])
    robot1.execute_plan(plan)
    rospy.sleep(.25)

    robot1.insert_key_better(transforms.task_board_T_key4)
    robot1.self_centering_grasp(5)
    robot1.turn_key()
    rospy.sleep(.25)

    if not use_strategy_b:
        robot1.gripper.ungrasp()
        plan = robot1.plan_tb_goals([transforms.task_board_T_key5])
        robot1.execute_plan(plan)
        rospy.sleep(.25)

        plan = robot1.plan_tb_goals([transforms.task_board_T_batt_case])
        robot1.gripper.go_to_closed_pos()
        robot1.execute_plan(plan)

    else:
        plan = robot1.plan_tb_goals([transforms.task_board_T_key5, transforms.task_board_T_key6])
        robot1.execute_plan(plan)
        robot1.gripper.ungrasp()
        rospy.sleep(0.25)

        plan = robot1.plan_tb_goals([transforms.task_board_T_key5, transforms.task_board_T_batt_case])
        robot1.gripper.go_to_closed_pos()
        robot1.execute_plan(plan)

    time_start = rospy.get_time()
    while (rospy.get_time() - time_start) < 1.5:
        robot1.force_control(k=0.00025, fz=-10, v_des=np.array([0, 0, 0]))
        rate.sleep()
    time_start = rospy.get_time()
    while (rospy.get_time() - time_start) < 3.5:
        robot1.force_control(k=0.00025, fz=-10, v_des=-x_vec*0.02)
        rate.sleep()
    robot1.arm.jog(0)
    rospy.sleep(.25)

    robot1.go_to_start()
    rospy.sleep(.25)
    robot1.gripper.set_grasp_position(1300)

    if not use_strategy_b:
        plan = robot1.plan_tb_goals([transforms.task_board_T_eth1, transforms.task_board_T_eth2,
                                     transforms.task_board_T_eth3])
        # raw_input("Continue")
        robot1.execute_plan(plan)
        robot1.self_centering_grasp(3, force=40)

        plan = robot1.plan_tb_goals([transforms.task_board_T_eth4, transforms.task_board_T_eth5])
        # raw_input("Continue")
        robot1.execute_plan(plan)
        rospy.sleep(.25)

        robot1.insert_plug(transforms.task_board_T_eth5, 0.241)
    else:
        plan = robot1.plan_tb_goals([transforms.task_board_T_eth1b, transforms.task_board_T_eth2b,
                                     transforms.task_board_T_eth3b, transforms.task_board_T_eth4b])
        # raw_input("Continue")
        robot1.execute_plan(plan)
        robot1.self_centering_grasp(3, force=40)

        plan = robot1.plan_tb_goals([transforms.task_board_T_eth5b, transforms.task_board_T_eth6b])
        # raw_input("Continue")
        robot1.execute_plan(plan)
        rospy.sleep(.25)

        robot1.insert_plug(transforms.task_board_T_eth6b, 0.266)

    rospy.sleep(.25)
    robot1.gripper.ungrasp()
    robot1.go_to_start()

    plan = robot1.plan_tb_goals([transforms.task_board_T_button_red])
    robot1.execute_plan(plan)
    robot1.press_button()
    rospy.sleep(.25)

    robot1.go_to_start()

