#!/usr/bin/env python
# import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Twist
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from sensor_msgs.msg import JointState

class ur5:
    def __init__(self):
        self.joints_vel_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
        moveit_commander.roscpp_initialize('')

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        self.traj_controller_enabled = True

        self.switch_controller_proxy = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)

    def enable_vel_controller(self):
        if not self.traj_controller_enabled:
            return
        rospy.wait_for_service('controller_manager/switch_controller')
        try:
            request = SwitchControllerRequest()
            request.start_controllers = ['joint_group_vel_controller']
            request.stop_controllers = ['scaled_pos_joint_traj_controller']
            request.strictness = request.BEST_EFFORT
            self.switch_controller_proxy(request)
            self.traj_controller_enabled = False
        except:
            print("Error while attempting to switch controllers")

    def enable_traj_controller(self):
        if self.traj_controller_enabled:
            return
        rospy.wait_for_service('controller_manager/switch_controller')
        try:
            request = SwitchControllerRequest()
            request.start_controllers = ['scaled_pos_joint_traj_controller']
            request.stop_controllers = ['joint_group_vel_controller']
            request.strictness = request.BEST_EFFORT
            self.switch_controller_proxy(request)
            self.traj_controller_enabled = True
        except:
            print("Error while attempting to switch controllers")

    def jog(self, desired_twist):
        self.enable_vel_controller()
        if np.count_nonzero(desired_twist) > 0:
            joints_set_point = np.array(self.move_group.get_current_joint_values())
            jacobian = self.move_group.get_jacobian_matrix(joints_set_point.tolist())
            pseudo_inverse_jacobian = np.linalg.pinv(jacobian)
            joint_velocities = pseudo_inverse_jacobian.dot(desired_twist)
            self.joints_vel_pub.publish(Float64MultiArray(data=joint_velocities))
        else:
            self.joints_vel_pub.publish(Float64MultiArray(data=[0, 0, 0, 0, 0, 0]))