import tf
import numpy as np
import geometry_msgs
# Convention: a_T_b means the coordinate of sytem of b defined with respect to the basis a

ee_T_camera_depth_optical_frame = tf.transformations.quaternion_matrix([0.383496451351805878,
                                                                        0.367950359621765699,
                                                                        0.602472824515935090,
                                                                        0.595457387538836813])
ee_T_camera_depth_optical_frame[0:3, 3] = [0.118684350170066136,
                                           -0.018676671764717767,
                                           -0.066594241264755188]

ee_T_camera_color_optical_frame = tf.transformations.euler_matrix(0.0400, 1.0999, 1.5383)
ee_T_camera_color_optical_frame[0:3, 3] = [-0.9570, 0.0574, -0.5960]

# t_comp = np.identity(4)
# t_comp[0:3, 3] = [-0.005, -0.005, 0.005]
# ee_T_camera_depth_optical_frame = ee_T_camera_depth_optical_frame.dot(t_comp)

# This is the results based on measuring and from solidworks
# ee_T_gripper = np.array([[0., 0., 1., 0.067],
#                               [-1., 0., 0., 0.],
#                               [0., -1., 0., 0.],
#                               [0., 0., 0., 1.]])
#
# gripper_T_camera_depth_optical_frame = np.array([[-1., 0., 0., 0.0175],
#                                                       [0., -0.90630779, -0.42261826, 0.07122917],
#                                                       [-0., -0.42261826, 0.90630779, 0.03935497],
#                                                       [0., 0., 0., 1.]])
# ee_T_camera_depth_optical_frame =np.dot(ee_T_gripper, gripper_T_camera_depth_optical_frame)

# This is for the hand, not the gripper, and is based on solidworks
# ee_T_camera_depth_optical_frame = np.array([[0., -0.1736453, 0.98480826, 0.13767881],
#                                                 [-0.70703996, -0.6964304, -0.12279737, 0.05167895],
#                                                 [0.70717359, -0.6962988, -0.12277416, 0.03751711],
#                                                 [0., 0., 0., 1.]])

camera_depth_optical_frame_T_camera_link = np.array([[0, -1, 0, 0],
                                                          [0, 0, -1, 0],
                                                          [1, 0, 0, 0],
                                                          [0, 0, 0, 1]])

camera_color_optical_frame_T_camera_link = np.array([[2.41661719e-03, -9.99987119e-01, -4.46344577e-03,
                                                      1.47872033e-02],
                                                     [3.14520478e-03, 4.47103743e-03, -9.99985059e-01,
                                                      -7.90057181e-05],
                                                     [9.99992134e-01, 2.40254263e-03, 3.15596905e-03,
                                                      1.33634703e-04],
                                                     [0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                                                      1.00000000e+00]])

ee_T_camera_link = np.dot(ee_T_camera_color_optical_frame,
                          camera_color_optical_frame_T_camera_link)

# In the below vectors, tb_norm1 is the outward facing normal vector to the right face of the task board
# tb_norm2 is the outward facing normal vector to the back face of the task board
task_board_T_tb_norm1 = np.array([0.97422141, -0.20275674, 0.09890573])
task_board_T_tb_norm2 = np.array([-0.22196383, -0.93984407, 0.25966337])
task_board_T_base_link_z = np.array([0.04063104, -0.27499046, -0.96058803])

icp_T_probe1_trans = [-0.141, -0.180, 0.156]
icp_T_probe1_rot = [0.685, 0.098, 0.719, 0.068]
icp_T_probe1 = tf.transformations.quaternion_matrix(icp_T_probe1_rot)
icp_T_probe1[0:3, 3] = icp_T_probe1_trans

icp_T_probe2_trans = [-0.145, -0.147, 0.269]
icp_T_probe2_rot = [0.685, 0.098, 0.719, 0.068]
icp_T_probe2 = tf.transformations.quaternion_matrix(icp_T_probe2_rot)
icp_T_probe2[0:3, 3] = icp_T_probe2_trans

icp_T_probe3_trans = [0.040, -0.187, 0.288]
icp_T_probe3_rot = [0.685, 0.098, 0.719, 0.068]
icp_T_probe3 = tf.transformations.quaternion_matrix(icp_T_probe3_rot)
icp_T_probe3[0:3, 3] = icp_T_probe3_trans

icp_T_probe4_trans = [0.152, -0.208, 0.300]
icp_T_probe4_rot = [0.535, 0.585, 0.430, -0.431]
icp_T_probe4 = tf.transformations.quaternion_matrix(icp_T_probe4_rot)
icp_T_probe4[0:3, 3] = icp_T_probe4_trans

icp_T_probe5_trans = [0.206, -0.024, 0.248]
icp_T_probe5_rot = [0.535, 0.585, 0.430, -0.431]
icp_T_probe5 = tf.transformations.quaternion_matrix(icp_T_probe5_rot)
icp_T_probe5[0:3, 3] = icp_T_probe5_trans

# task_board_T_button_trans = [0.062, -0.141, 0.117]
# task_board_T_button_rot = [0.605, 0.500, 0.515, -0.343]
# task_board_T_button = tf.transformations.quaternion_matrix(task_board_T_button_rot)
# task_board_T_button[0:3, 3] = task_board_T_button_trans
#
# task_board_T_key1_trans = [-0.081, -0.074, 0.122]
# task_board_T_key1_rot = [0.672, 0.018, 0.723, 0.159]
# task_board_T_key1 = tf.transformations.quaternion_matrix(task_board_T_key1_rot)
# task_board_T_key1[0:3, 3] = task_board_T_key1_trans
#
# task_board_T_key2_trans = [-0.082, -0.060, 0.171]
# task_board_T_key2_rot = [0.672, 0.018, 0.723, 0.159]
# task_board_T_key2 = tf.transformations.quaternion_matrix(task_board_T_key2_rot)
# task_board_T_key2[0:3, 3] = task_board_T_key2_trans
#
# task_board_T_key3_trans = [-0.066, -0.023, 0.110]
# task_board_T_key3_rot = [0.616, 0.486, 0.517, -0.344]
# task_board_T_key3 = tf.transformations.quaternion_matrix(task_board_T_key3_rot)
# task_board_T_key3[0:3, 3] = task_board_T_key3_trans

task_board_T_pre_button_trans = [0.030, -0.1465, 0.355]
task_board_T_pre_button_rot = [0.489, 0.525, -0.497, 0.488]
task_board_T_pre_button = tf.transformations.quaternion_matrix(task_board_T_pre_button_rot)
task_board_T_pre_button[0:3, 3] = task_board_T_pre_button_trans

task_board_T_button_trans = [-0.082, -0.061, 0.355]
task_board_T_button_rot = [0.489, 0.525, -0.497, 0.488]
task_board_T_button = tf.transformations.quaternion_matrix(task_board_T_button_rot)
task_board_T_button[0:3, 3] = task_board_T_button_trans

task_board_T_key1_trans = [-0.242, -0.090, 0.353]
task_board_T_key1_rot = [-0.007, 0.718, 0.015, 0.696]
task_board_T_key1 = tf.transformations.quaternion_matrix(task_board_T_key1_rot)
task_board_T_key1[0:3, 3] = task_board_T_key1_trans

task_board_T_key2_trans = [-0.242, -0.092, 0.318]
task_board_T_key2_rot = [-0.007, 0.718, 0.015, 0.696]
task_board_T_key2 = tf.transformations.quaternion_matrix(task_board_T_key2_rot)
task_board_T_key2[0:3, 3] = task_board_T_key2_trans

task_board_T_key3_trans = [-0.242, -0.092, 0.364]
task_board_T_key3_rot = [-0.007, 0.718, 0.015, 0.696]
task_board_T_key3 = tf.transformations.quaternion_matrix(task_board_T_key3_rot)
task_board_T_key3[0:3, 3] = task_board_T_key3_trans

task_board_T_key4_trans = [-0.2375, -0.1465, 0.354]
task_board_T_key4_rot = [0.481, 0.533, -0.460, 0.522]
task_board_T_key4 = tf.transformations.quaternion_matrix(task_board_T_key4_rot)
task_board_T_key4[0:3, 3] = task_board_T_key4_trans

task_board_T_key5_trans = [-0.2375, -0.1465, 0.394]
task_board_T_key5_rot = [0.481, 0.533, -0.460, 0.522]
task_board_T_key5 = tf.transformations.quaternion_matrix(task_board_T_key5_rot)
task_board_T_key5[0:3, 3] = task_board_T_key5_trans

task_board_T_key6_trans = [-0.2375, -0.200, 0.394]
task_board_T_key6_rot = [0.481, 0.533, -0.460, 0.522]
task_board_T_key6 = tf.transformations.quaternion_matrix(task_board_T_key6_rot)
task_board_T_key6[0:3, 3] = task_board_T_key6_trans

probe_point_4_des = np.array([-0.23407, -0.09202, 0.31794])

task_board_T_batt_case_trans = [-0.164, -0.087, 0.327]
task_board_T_batt_case_rot = [0.498, 0.518, -0.476, 0.507]
task_board_T_batt_case = tf.transformations.quaternion_matrix(task_board_T_batt_case_rot)
task_board_T_batt_case[0:3, 3] = task_board_T_batt_case_trans

task_board_T_button_red_trans = [-0.104, -0.061, 0.355]
task_board_T_button_red_rot = [0.489, 0.525, -0.497, 0.488]
task_board_T_button_red = tf.transformations.quaternion_matrix(task_board_T_button_red_rot)
task_board_T_button_red[0:3, 3] = task_board_T_button_red_trans

base_link_T_start = geometry_msgs.msg.Pose()
base_link_T_start.orientation.x = 0.612597951797
base_link_T_start.orientation.y = 0.633871311828
base_link_T_start.orientation.z = -0.330204353115
base_link_T_start.orientation.w = 0.337484806591
base_link_T_start.position.x = 0.0552851616652
base_link_T_start.position.y = 0.306774550115
base_link_T_start.position.z = 0.597339559252

start_joint_angles = np.array([0.9351882934570312,
                              -2.0584309736834925,
                              1.6450657844543457,
                              -1.6368029753314417,
                              -1.2172115484820765,
                              2.563282012939453])

task_board_T_eth1_trans = [0.012, -0.154, 0.418]
task_board_T_eth1_rot = [0.009, 0.964, -0.000, 0.264]
task_board_T_eth1 = tf.transformations.quaternion_matrix(task_board_T_eth1_rot)
task_board_T_eth1[0:3, 3] = task_board_T_eth1_trans

task_board_T_eth2_trans = [0.006, -0.152, 0.2615]
task_board_T_eth2_rot = [0.009, 0.964, -0.000, 0.264]
task_board_T_eth2 = tf.transformations.quaternion_matrix(task_board_T_eth2_rot)
task_board_T_eth2[0:3, 3] = task_board_T_eth2_trans

task_board_T_eth3_trans = [-0.029, -0.152, 0.262]
task_board_T_eth3_rot = [0.009, 0.964, -0.000, 0.264]
task_board_T_eth3 = tf.transformations.quaternion_matrix(task_board_T_eth3_rot)
task_board_T_eth3[0:3, 3] = task_board_T_eth3_trans

task_board_T_eth4_trans = [-0.029, -0.152, 0.277]
task_board_T_eth4_rot = [0.009, 0.964, -0.000, 0.264]
task_board_T_eth4 = tf.transformations.quaternion_matrix(task_board_T_eth4_rot)
task_board_T_eth4[0:3, 3] = task_board_T_eth4_trans

task_board_T_eth5_trans = [-0.049, -0.162, 0.247]
task_board_T_eth5_rot = [0.040, 0.986, -0.011, 0.161]
task_board_T_eth5 = tf.transformations.quaternion_matrix(task_board_T_eth5_rot)
task_board_T_eth5[0:3, 3] = task_board_T_eth5_trans

task_board_T_eth1b_trans = [-0.297, -0.161, 0.396]
task_board_T_eth1b_rot = [0.965, 0.048, -0.258, 0.031]
task_board_T_eth1b = tf.transformations.quaternion_matrix(task_board_T_eth1b_rot)
task_board_T_eth1b[0:3, 3] = task_board_T_eth1b_trans

task_board_T_eth2b_trans = [-0.298, -0.161, 0.304]
task_board_T_eth2b_rot = [0.965, 0.048, -0.258, 0.031]
task_board_T_eth2b = tf.transformations.quaternion_matrix(task_board_T_eth2b_rot)
task_board_T_eth2b[0:3, 3] = task_board_T_eth2b_trans

task_board_T_eth3b_trans = [-0.267, -0.161, 0.304]
task_board_T_eth3b_rot = [0.965, 0.048, -0.258, 0.031]
task_board_T_eth3b = tf.transformations.quaternion_matrix(task_board_T_eth3b_rot)
task_board_T_eth3b[0:3, 3] = task_board_T_eth3b_trans

task_board_T_eth4b_trans = [-0.267, -0.161, 0.263]
task_board_T_eth4b_rot = [0.965, 0.048, -0.258, 0.031]
task_board_T_eth4b = tf.transformations.quaternion_matrix(task_board_T_eth4b_rot)
task_board_T_eth4b[0:3, 3] = task_board_T_eth4b_trans

task_board_T_eth5b_trans = [-0.267, -0.161, 0.285]
task_board_T_eth5b_rot = [0.965, 0.048, -0.258, 0.031]
task_board_T_eth5b = tf.transformations.quaternion_matrix(task_board_T_eth5b_rot)
task_board_T_eth5b[0:3, 3] = task_board_T_eth5b_trans

task_board_T_eth6b_trans = [-0.309, -0.152, 0.275]
task_board_T_eth6b_rot = [0.965, 0.022, -0.258, 0.024]
task_board_T_eth6b = tf.transformations.quaternion_matrix(task_board_T_eth6b_rot)
task_board_T_eth6b[0:3, 3] = task_board_T_eth6b_trans