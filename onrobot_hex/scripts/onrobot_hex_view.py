import rospy
from onrobot_hex_client.driver import onrobot_hex
if __name__ == "__main__":
    rospy.init_node("onrobot_hex_view", anonymous=True)
    force_sensor = onrobot_hex()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        fx, fy, fz, tx, ty, tz = force_sensor.get()
        print("Fx, Fy, Fz, Tx, Ty, Tz:")
        print(fx, fy, fz, tx, ty, tz)
        rate.sleep()
