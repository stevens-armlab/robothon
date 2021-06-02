from sensor_msgs.msg import PointCloud2, PointField
from registration import icp_registration
import rospy
import tf
import numpy as np

class d435i:
    def __init__(self):
        self.pcl_topic = '/camera/depth/color/points'
        self.latest_ros_cloud = None
        self.registration = icp_registration()
        rospy.Subscriber(self.pcl_topic, PointCloud2, self.ros_pcl_callback)

    def ros_pcl_callback(self, ros_cloud):
        self.latest_ros_cloud = ros_cloud

    def get_transform(self):
        t = self.registration.get_transform(self.latest_ros_cloud)
        return t

    def show_transform(self):
        br = tf.TransformBroadcaster()
        mat = np.linalg.inv(self.get_transform())
        trans = tf.transformations.translation_from_matrix(mat)
        quat = tf.transformations.quaternion_from_matrix(mat)
        br.sendTransform(trans, quat, rospy.Time.now(), "icp", "camera_depth_optical_frame")