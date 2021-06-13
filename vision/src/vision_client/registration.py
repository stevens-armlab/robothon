import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from ctypes import *  # convert float to uint32
import numpy as np
import copy

class icp_registration:
    def __init__(self):
        self.voxel_size = 0.01
        self.max_iteration_num = 100
        self.source_path = "/home/kevin/ros_ws/robothon_ws/src/robothon/vision/point_clouds/taskboard_source.pcd"
        self.source = o3d.io.read_point_cloud(self.source_path).voxel_down_sample(0.001)
        self.source_down, self.source_fpfh = self.preprocess_point_cloud(self.source, self.voxel_size)

    def get_transform(self, ros_cloud, view_pcl_overlay=False):
        target = self.convertCloudFromRosToOpen3d(ros_cloud)
        target = target.voxel_down_sample(voxel_size=0.001)
        target = self.remove_points_from_table(target)
        target_down, target_fpfh = self.preprocess_point_cloud(target, self.voxel_size)

        T_w_i = None
        for i in range(100):
            result_fast = self.execute_fast_global_registration(self.source_down, target_down,
                                                                self.source_fpfh, target_fpfh,
                                                                self.voxel_size)
            icp_init = result_fast.transformation

            T_w_i, fitness, rmse = self.pairwise_registration(self.source, target, icp_init)

            print("Registration Attempt #%d" % i)
            print("ICP Fitness: %f", fitness)
            print("ICP RMSE: %f", rmse)
            if fitness > 0.92:
                break

        if view_pcl_overlay:
            self.visualize_combined_point_clouds(self.source, target, T_w_i)
        return T_w_i, self.source, target

    # https://github.com/felixchenfy/open3d_ros_pointcloud_conversion.git
    @staticmethod
    def convertCloudFromRosToOpen3d(ros_cloud):
        # The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
        FIELDS_XYZ = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        FIELDS_XYZRGB = FIELDS_XYZ + \
                        [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

        # Bit operations
        BIT_MOVE_16 = 2 ** 16
        BIT_MOVE_8 = 2 ** 8
        convert_rgbUint32_to_tuple = lambda rgb_uint32: (
            (rgb_uint32 & 0x00ff0000) >> 16, (rgb_uint32 & 0x0000ff00) >> 8, (rgb_uint32 & 0x000000ff)
        )
        convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
            int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
        )

        # Get cloud data from ros_cloud
        field_names = [field.name for field in ros_cloud.fields]
        cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names=field_names))

        # Check empty
        open3d_cloud = o3d.geometry.PointCloud()
        if len(cloud_data) == 0:
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD = 3  # x, y, z, rgb

            # Get xyz
            xyz = [(x, y, z) for x, y, z, rgb in cloud_data]  # (why cannot put this line below rgb?)

            # Get rgb
            # Check whether int or float
            if type(cloud_data[0][IDX_RGB_IN_FIELD]) == float:  # if float (from pcl::toROSMsg)
                rgb = [convert_rgbFloat_to_tuple(rgb) for x, y, z, rgb in cloud_data]
            else:
                rgb = [convert_rgbUint32_to_tuple(rgb) for x, y, z, rgb in cloud_data]

            # combine
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb) / 255.0)
        else:
            xyz = [(x, y, z) for x, y, z in cloud_data]  # get xyz
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

        # return
        return open3d_cloud

    @staticmethod
    def remove_points_from_table(pcd, visualize=False):

        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                                 ransac_n=10,
                                                 num_iterations=1000)

        taskboard = pcd.select_down_sample(inliers, invert=True)

        if visualize:
            o3d.visualization.draw_geometries([taskboard])

        return taskboard

    def pairwise_registration(self, source, target, icp_init):
        max_correspondence_distance_coarse = self.voxel_size * 15
        max_correspondence_distance_fine = self.voxel_size * 1.5

        source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
        target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))

        icp_coarse = o3d.registration.registration_icp(
            source, target, max_correspondence_distance_coarse, icp_init,
            o3d.registration.TransformationEstimationPointToPlane(),
            o3d.registration.ICPConvergenceCriteria(max_iteration=self.max_iteration_num))

        icp_fine = o3d.registration.registration_icp(
            source, target, max_correspondence_distance_fine,
            icp_coarse.transformation,
            o3d.registration.TransformationEstimationPointToPlane(),
            o3d.registration.ICPConvergenceCriteria(max_iteration=self.max_iteration_num))

        transformation_icp = icp_fine.transformation
        fitness = icp_fine.fitness
        return np.linalg.inv(transformation_icp), fitness, icp_fine.inlier_rmse

    @staticmethod
    def visualize_combined_point_clouds(source, target, T_w_i):
        transformed_target = copy.deepcopy(target).transform(T_w_i)
        o3d.visualization.draw_geometries([source, transformed_target])

    @staticmethod
    def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                         target_fpfh, voxel_size):
        distance_threshold = voxel_size * 0.5

        result = o3d.registration.registration_fast_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            o3d.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=distance_threshold))
        return result

    @staticmethod
    def preprocess_point_cloud(pcd, voxel_size):
        pcd_down = pcd.voxel_down_sample(voxel_size)

        radius_normal = voxel_size * 2
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = voxel_size * 5
        pcd_fpfh = o3d.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh



