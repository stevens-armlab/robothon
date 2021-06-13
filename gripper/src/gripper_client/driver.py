import rospy
from std_msgs.msg import Float64, Int8, Bool
from gripper.msg import grasp


class gripper_driver:
    def __init__(self):
        self.grasp_set_pos_pub = rospy.Publisher('gripper/grasp/set_position', Float64, queue_size=10)
        self.start_grasp_pub = rospy.Publisher('gripper/grasp/start_grasp', grasp, queue_size=10)
        self.stop_grasp_pub = rospy.Publisher('gripper/grasp/stop', Bool, queue_size=10)
        self.grasp_load_sub = rospy.Subscriber('gripper/grasp/load', Float64, self.grasp_load_cb)
        self.grasp_state_sub = rospy.Subscriber('gripper/grasp/state', Int8, self.grasp_state_cb)

        self.current_load = None
        self.grasp_state = None

    def grasp(self, speed=20, force=5, force_threshold=90, pos_max=2600):
        grasp_msg = grasp(speed=speed, force=force, force_threshold=force_threshold, pos_max=pos_max)
        self.start_grasp_pub.publish(grasp_msg)

    def ungrasp(self, position=1200):
        self.set_grasp_position(position)

    def set_grasp_position(self, position=1200):
        grasp_pos_setpoint = Float64(data=position)
        self.grasp_set_pos_pub.publish(grasp_pos_setpoint)

    def stop_grasp(self):
        self.stop_grasp_pub.publish(Bool(True))

    def grasp_load_cb(self, load_msg):
        self.current_load = load_msg.data

    def grasp_state_cb(self, state_msg):
        self.grasp_state = state_msg.data

    def go_to_closed_pos(self):
        self.set_grasp_position(position=2290)







