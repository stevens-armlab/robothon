import rospy

from onrobot_hex_client.driver import onrobot_hex
from ur5_client.ur5 import ur5
from hand_client.driver import hand_driver
import numpy as np

class robot:
    def __init__(self):
        self.arm = ur5()
        self.force_sensor = onrobot_hex()
        self.hand = hand_driver()

    def press_button(self):
        rate = rospy.Rate(10)
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
