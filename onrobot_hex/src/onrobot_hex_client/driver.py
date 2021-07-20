import logging
import thread
import os
import rospy
import rospkg
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import threading

class onrobot_hex:
    def __init__(self):
        print(os.getcwd())
        self.host = '192.168.1.11'
        self.port = 30004
        self.frequency = 125
        self.config = rospkg.RosPack().get_path('onrobot_hex') + '/config/record_configuration.xml'
        self.verbose = False
        self.buffered = False
        self.binary = False

        self.hex_data = WrenchStamped()
        self._fx = None
        self._fy = None
        self._fz = None
        self._tx = None
        self._ty = None
        self._tz = None

        # get ur5 joint data at same timestamp as onrobot hex data
        # (not currently used, but functionality might be nice one day)
        # these values can be found in onrobot_hex/config/record_configuration.xml
        # an example where some values are uncommented is in record_configuration_publish_q.xml
        # uncommend values you want to use
        # self.ur5_data = JointState()
        # self._actual_q = None #VECTOR6D
        # self._actual_qd = None #VECTOR6D
        # self._target_q = None #VECTOR6D
        # self._target_qd = None #VECTOR6D
        # other possible unused options
        # self._target_qdd = None #VECTOR6D
        # self._actual_current = None #VECTOR6D
        # self._joint_control_output = None #VECTOR6D
        # self._actual_TCP_pose = None #VECTOR6D
        # self._actual_TCP_speed = None #VECTOR6D
        # self._actual_TCP_force = None #VECTOR6D
        # self._speed_scaling = None #DOUBLE
        # self._target_speed_fraction = None #DOUBLE

        self.lock = threading.Lock()
        self.client_thread = threading.Thread(target=self.rtde_client)
        self.client_thread.start()
        self.stop_thread = False

        self._reference_zero = [0, 0, 0, 0, 0, 0]
        self._zero_flag = False

        rospy.on_shutdown(self.close)

    def get_fx(self):
        with self.lock:
            return self._fx

    def get_fy(self):
        with self.lock:
            return self._fy

    def get_fz(self):
        with self.lock:
            return self._fz

    def get_tx(self):
        with self.lock:
            return self._tx

    def get_ty(self):
        with self.lock:
            return self._ty

    def get_tz(self):
        with self.lock:
            return self._tz

    def get_force(self):
        with self.lock:
            return self._fx, self._fy, self._fz

    def get_torque(self):
        with self.lock:
            return self._tx, self._ty, self._tz

    def get(self):
        with self.lock:
            return self._fx, self._fy, self._fz, self._tx, self._ty, self._tz

    def close(self):
        self.stop_thread = True

    def zero(self):
        with self.lock:
            self._zero_flag = True

    def rtde_client(self):
        hex_pub = rospy.Publisher('onrobot/hex_data', WrenchStamped, queue_size=1)
        # uncomment if you want to publish rtde q state and q target values as described in the the init
        # ur5_data_pub = rospy.Publisher('onrobot/ur5_state', JointState, queue_size=1)
        # ur5_target_pub = rospy.Publisher('onrobot/ur5_target', JointState, queue_size=1)
        # ur5_speed_scale_pub = rospy.Publisher('onrobot/ur5_speed_scale', Float64, queue_size=1)

        if self.verbose:
            logging.basicConfig(level=logging.INFO)

        conf = rtde_config.ConfigFile(self.config)
        output_names, output_types = conf.get_recipe('out')

        con = rtde.RTDE(self.host, self.port)
        con.connect()

        # get controller version
        con.get_controller_version()

        # setup recipes
        if not con.send_output_setup(output_names, output_types, frequency=self.frequency):
            logging.error('Unable to configure output')
            thread.exit()

        # start data synchronization
        if not con.send_start():
            logging.error('Unable to start synchronization')
            thread.exit()

        keep_running = True

        # Used so that publishing to topics can occur outside of mutex lock
        hex_data = WrenchStamped()
        # ur5_data = JointState()
        # ur5_target = JointState()
        # speed_scale = Float64

        while keep_running:
            try:
                if self.buffered:
                    state = con.receive_buffered(self.binary)
                else:
                    state = con.receive(self.binary)
                if state is not None:
                    with self.lock:
                        if self._zero_flag:
                            self._reference_zero[0] = state.input_double_register_30
                            self._reference_zero[1] = state.input_double_register_31
                            self._reference_zero[2] = state.input_double_register_32
                            self._reference_zero[3] = state.input_double_register_33
                            self._reference_zero[4] = state.input_double_register_34
                            self._reference_zero[5] = state.input_double_register_35
                            self._fx = 0
                            self._fy = 0
                            self._fz = 0
                            self._tx = 0
                            self._ty = 0
                            self._tz = 0
                            hex_data.header.stamp = rospy.get_rostime()
                            hex_data.wrench.force.x = self._fx
                            hex_data.wrench.force.y = self._fy
                            hex_data.wrench.force.z = self._fz
                            hex_data.wrench.torque.x = self._tx
                            hex_data.wrench.torque.y = self._ty
                            hex_data.wrench.torque.z = self._tz
                            self._zero_flag = False
                        else:
                            self._fx = state.input_double_register_30 - self._reference_zero[0]
                            self._fy = state.input_double_register_31 - self._reference_zero[1]
                            self._fz = state.input_double_register_32 - self._reference_zero[2]
                            self._tx = state.input_double_register_33 - self._reference_zero[3]
                            self._ty = state.input_double_register_34 - self._reference_zero[4]
                            self._tz = state.input_double_register_35 - self._reference_zero[5]
                            hex_data.header.stamp = rospy.get_rostime()
                            hex_data.wrench.force.x = self._fx
                            hex_data.wrench.force.y = self._fy
                            hex_data.wrench.force.z = self._fz
                            hex_data.wrench.torque.x = self._tx
                            hex_data.wrench.torque.y = self._ty
                            hex_data.wrench.torque.z = self._tz

                        # uncomment if you want to publish q target and q state values from rtde
                        # self._actual_q = state.actual_q  # VECTOR6D
                        # self._actual_qd = state.actual_qd  # VECTOR6D
                        # self._target_q = state.target_q  # VECTOR6D
                        # self._target_qd = state.target_qd  # VECTOR6D
                        # ur5_target.header.stamp = current_time
                        # ur5_target.position = self._target_q
                        # ur5_target.velocity = self._target_qd
                        # ur5_data.header.stamp = current_time
                        # ur5_data.position = self._actual_q
                        # ur5_data.velocity = self._actual_qd
                        # speed_scale = state.speed_scaling  # DOUBLE

                        # self._target_qdd = state.target_qdd  # VECTOR6D
                        # self._actual_current = state.actual_current  # VECTOR6D
                        # self._joint_control_output = state.joint_control_output  # VECTOR6D
                        # self._actual_TCP_pose = state.actual_TCP_pose  # VECTOR6D
                        # self._actual_TCP_speed = state.actual_TCP_speed  # VECTOR6D
                        # self._actual_TCP_force = state.actual_TCP_force  # VECTOR6D

                    hex_pub.publish(hex_data)
                    # uncomment if you want to publish q target and q state values from rtde
                    # ur5_data_pub.publish(ur5_data)
                    # ur5_target_pub.publish(ur5_target)
                    # ur5_speed_scale_pub.publish(speed_scale)

                if self.stop_thread:
                    keep_running = False
            except rtde.RTDEException:
                con.disconnect()
                thread.exit()
        print('Force Sensor RTDE Client Disconnecting')
        con.send_pause()
        con.disconnect()




