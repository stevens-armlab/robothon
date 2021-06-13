import logging
import thread
import os
import rospy
import rospkg
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
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
        self._fx = None
        self._fy = None
        self._fz = None
        self._tx = None
        self._ty = None
        self._tz = None

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
        pub_fx = rospy.Publisher('onrobot/fx', Float64, queue_size=1)
        pub_fy = rospy.Publisher('onrobot/fy', Float64, queue_size=1)
        pub_fz = rospy.Publisher('onrobot/fz', Float64, queue_size=1)
        pub_tx = rospy.Publisher('onrobot/tx', Float64, queue_size=1)
        pub_ty = rospy.Publisher('onrobot/ty', Float64, queue_size=1)
        pub_tz = rospy.Publisher('onrobot/tz', Float64, queue_size=1)

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
        local_force_vars = [0, 0, 0, 0, 0, 0]

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
                            local_force_vars = [0, 0, 0, 0, 0, 0]
                            self._zero_flag = False
                        else:
                            self._fx = state.input_double_register_30 - self._reference_zero[0]
                            self._fy = state.input_double_register_31 - self._reference_zero[1]
                            self._fz = state.input_double_register_32 - self._reference_zero[2]
                            self._tx = state.input_double_register_33 - self._reference_zero[3]
                            self._ty = state.input_double_register_34 - self._reference_zero[4]
                            self._tz = state.input_double_register_35 - self._reference_zero[5]
                            local_force_vars = [self._fx, self._fy, self._fz, self._tx, self._ty, self._tz]
                    pub_fx.publish(local_force_vars[0])
                    pub_fy.publish(local_force_vars[1])
                    pub_fz.publish(local_force_vars[2])
                    pub_tx.publish(local_force_vars[3])
                    pub_ty.publish(local_force_vars[4])
                    pub_tz.publish(local_force_vars[5])
                if self.stop_thread:
                    keep_running = False
            except rtde.RTDEException:
                con.disconnect()
                thread.exit()
        print('Force Sensor RTDE Client Disconnecting')
        con.send_pause()
        con.disconnect()




