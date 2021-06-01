import rospy
from std_msgs.msg import Float64, Int8, Bool
from hand.msg import grasp

class hand_driver:
    def __init__(self):
        self.spread_pub = rospy.Publisher('hand/spread/set_position', Float64, queue_size=10)
        self.grasp_set_pos_pub = rospy.Publisher('hand/grasp/set_position', Float64, queue_size=10)
        self.start_grasp_pub = rospy.Publisher('hand/grasp/start_grasp', grasp, queue_size=10)
        self.stop_grasp_pub = rospy.Publisher('hand/grasp/stop', Bool, queue_size=10)
        self.grasp_load_sub = rospy.Subscriber('hand/grasp/load', Float64, self.grasp_load_cb)
        self.grasp_state_sub = rospy.Subscriber('hand/grasp/state', Int8, self.grasp_state_cb)

        self.current_load = None
        self.grasp_state = None
        self.spread_pos = 525.0
        self.unspread_pos = 708.0

    def grasp(self, speed=20, force=30, force_threshold=90, pos_max=3883):
        grasp_msg = grasp(speed=speed, force=force, force_threshold=force_threshold, pos_max=pos_max)
        self.start_grasp_pub.publish(grasp_msg)

    def ungrasp(self, position=1690):
        self.set_grasp_position(position)

    def set_grasp_position(self, position=1690):
        grasp_pos_setpoint = Float64(data=position)
        self.grasp_set_pos_pub.publish(grasp_pos_setpoint)

    def stop_grasp(self):
        self.stop_grasp_pub.publish(Bool(True))

    def set_spread(self, position):
        spread_pos_setpoint = Float64(data=position)
        self.spread_pub.publish(spread_pos_setpoint)

    def spread(self):
        self.set_spread(self.spread_pos)

    def unspread(self):
        self.set_spread(self.unspread_pos)

    def grasp_load_cb(self, load_msg):
        self.current_load = load_msg.data

    def grasp_state_cb(self, state_msg):
        self.grasp_state = state_msg.data






