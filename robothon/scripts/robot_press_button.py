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

    rospy.sleep(1)

    robot1.go_to_start()
    # rospy.sleep(1)
    # robot1.compute_task_board_registration()

    plan = robot1.plan_tb_goals([transforms.task_board_T_button])
    raw_input("Continue")
    robot1.execute_plan(plan)
    robot1.press_button()
    robot1.gripper.ungrasp()
    rospy.sleep(1)

    plan = robot1.plan_tb_goals([transforms.task_board_T_key1])
    raw_input("Continue")
    robot1.execute_plan(plan)
    rospy.sleep(1)

    plan = robot1.plan_tb_goals([transforms.task_board_T_key2])
    raw_input("Continue")
    robot1.execute_plan(plan)
    robot1.gripper.grasp()
    rospy.sleep(5)

    plan = robot1.plan_tb_goals([transforms.task_board_T_key3])
    raw_input("Continue")
    robot1.execute_plan(plan)
    robot1.gripper.grasp()
    rospy.sleep(1)

    plan = robot1.plan_tb_goals([transforms.task_board_T_key4])
    raw_input("Continue")
    robot1.execute_plan(plan)
    rospy.sleep(1)

    raw_input("Continue")
    robot1.insert_key()


