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
        self.fx = None
        self.fy = None
        self.fz = None
        self.tx = None
        self.ty = None
        self.tz = None

        self.lock = threading.Lock()
        self.client_thread = threading.Thread(target=self.rtde_client)
        self.client_thread.start()
        self.stop_thread = False

        rospy.on_shutdown(self.close)

    def get_fx(self):
        with self.lock:
            return self.fx

    def get_fy(self):
        with self.lock:
            return self.fy

    def get_fz(self):
        with self.lock:
            return self.fz

    def get_tx(self):
        with self.lock:
            return self.tx

    def get_ty(self):
        with self.lock:
            return self.ty

    def get_tz(self):
        with self.lock:
            return self.tz

    def get_force(self):
        with self.lock:
            return self.fx, self.fy, self.fz

    def get_torque(self):
        with self.lock:
            return self.tx, self.ty, self.tz

    def get(self):
        with self.lock:
            return self.fx, self.fy, self.fz, self.tx, self.ty, self.tz

    def close(self):
        self.stop_thread = True

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

        i = 1
        keep_running = True
        while keep_running:
            try:
                if self.buffered:
                    state = con.receive_buffered(self.binary)
                else:
                    state = con.receive(self.binary)
                if state is not None:
                    with self.lock:
                        self.fx = state.input_double_register_30
                        self.fy = state.input_double_register_31
                        self.fz = state.input_double_register_32
                        self.tx = state.input_double_register_33
                        self.ty = state.input_double_register_34
                        self.tz = state.input_double_register_35
                    pub_fx.publish(state.input_double_register_30)
                    pub_fy.publish(state.input_double_register_31)
                    pub_fz.publish(state.input_double_register_32)
                    pub_tx.publish(state.input_double_register_33)
                    pub_ty.publish(state.input_double_register_34)
                    pub_tz.publish(state.input_double_register_35)
                    i += 1
                if self.stop_thread:
                    keep_running = False
            except rtde.RTDEException:
                con.disconnect()
                thread.exit()
        print('Force Sensor RTDE Client Disconnecting')
        con.send_pause()
        con.disconnect()




