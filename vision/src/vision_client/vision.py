from sensor_msgs.msg import PointCloud2, PointField
from registration import icp_registration
import rospy
import tf
import numpy as np

class d435i:
    def __init__(self):
        self.pcl_topic = 'camera/depth/color/points'
        self.latest_ros_cloud = None
        self.registration = icp_registration()

    def get_transform(self):
        ros_pcl = rospy.wait_for_message(self.pcl_topic, PointCloud2)
        t, a, b = self.registration.get_transform(ros_pcl, view_pcl_overlay=False)
        return t, a, b

    def show_transform(self):
        br = tf.TransformBroadcaster()
        mat = np.linalg.inv(self.get_transform())
        trans = tf.transformations.translation_from_matrix(mat)
        quat = tf.transformations.quaternion_from_matrix(mat)
        br.sendTransform(trans, quat, rospy.Time.now(), "icp", "camera_depth_optical_frame")

    def capture_pcl(self):
        pcl = rospy.wait_for_message(self.pcl_topic, PointCloud2)
        return icp_registration.convertCloudFromRosToOpen3d(pcl)


