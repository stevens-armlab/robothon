import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from ctypes import *  # convert float to uint32
import numpy as np

class icp_registration:
    def __init__(self):
        self.voxel_size = 0.02
        self.source_path = "/home/kevin/ros_ws/robothon_ws/src/robothon/vision/point_clouds/complete_model.pcd"
        self.source_pcd = o3d.io.read_point_cloud(self.source_path)
        self.source_pcd = o3d.geometry.voxel_down_sample(self.source_pcd, voxel_size=0.002)

    def get_transform(self, ros_cloud, view_pcl_overlay=False):
        target_pcd = self.convertCloudFromRosToOpen3d(ros_cloud)
        target_pcd = o3d.geometry.voxel_down_sample(target_pcd, voxel_size=0.002)
        Twi, pose_graph = self.estimate([self.source_pcd, target_pcd])
        if view_pcl_overlay:
            self.visualize_combined_point_clouds([self.source_pcd, target_pcd], pose_graph)
        return Twi, self.source_pcd, target_pcd

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
        open3d_cloud = o3d.PointCloud()
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
            open3d_cloud.points = o3d.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = o3d.Vector3dVector(np.array(rgb) / 255.0)
        else:
            xyz = [(x, y, z) for x, y, z in cloud_data]  # get xyz
            open3d_cloud.points = o3d.Vector3dVector(np.array(xyz))

        # return
        return open3d_cloud

    def pairwise_registration(self, source, target):
        # print("Apply point-to-plane ICP")
        max_correspondence_distance_coarse = self.voxel_size * 15
        max_correspondence_distance_fine = self.voxel_size * 1.5

        o3d.geometry.estimate_normals(source, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
        o3d.geometry.estimate_normals(target, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))

        icp_coarse = o3d.registration.registration_icp(
            source, target, max_correspondence_distance_coarse, np.identity(4),
            o3d.registration.TransformationEstimationPointToPlane())
        icp_fine = o3d.registration.registration_icp(
            source, target, max_correspondence_distance_fine,
            icp_coarse.transformation,
            o3d.registration.TransformationEstimationPointToPlane())
        transformation_icp = icp_fine.transformation
        information_icp = o3d.registration.get_information_matrix_from_point_clouds(
            source, target, max_correspondence_distance_fine,
            icp_fine.transformation)
        return transformation_icp, information_icp

    def full_registration(self, pcds, max_correspondence_distance_coarse, max_correspondence_distance_fine):
        pose_graph = o3d.registration.PoseGraph()
        odometry = np.identity(4)
        pose_graph.nodes.append(o3d.registration.PoseGraphNode(odometry))
        n_pcds = len(pcds)
        for source_id in range(n_pcds):
            for target_id in range(source_id + 1, n_pcds):
                transformation_icp, information_icp = self.pairwise_registration(
                    pcds[source_id], pcds[target_id])
                # print("Build o3d.registration.PoseGraph")
                if target_id == source_id + 1:  # odometry case
                    odometry = np.dot(transformation_icp, odometry)
                    pose_graph.nodes.append(
                        o3d.registration.PoseGraphNode(
                            np.linalg.inv(
                                odometry)))  # odometry^-1 = Ti that is a pose matrix wich transform Pi into to global space Ti = Twi = T0i
                    pose_graph.edges.append(
                        o3d.registration.PoseGraphEdge(source_id,
                                                       target_id,
                                                       transformation_icp,
                                                       information_icp,
                                                       uncertain=False))
                else:  # loop closure case
                    pose_graph.edges.append(
                        o3d.registration.PoseGraphEdge(source_id,
                                                       target_id,
                                                       transformation_icp,
                                                       information_icp,
                                                       uncertain=True))
        return pose_graph

    def estimate(self, pcds):
        # print("Full registration starts")
        max_correspondence_distance_coarse = self.voxel_size * 15
        max_correspondence_distance_fine = self.voxel_size * 1.5

        pose_graph = self.full_registration(pcds, max_correspondence_distance_coarse, max_correspondence_distance_fine)
        # print("Optimizing PoseGraph by nonlinear optimization starts")
        option = o3d.registration.GlobalOptimizationOption(
            max_correspondence_distance=max_correspondence_distance_fine,
            edge_prune_threshold=0.25,
            reference_node=0)

        o3d.registration.global_optimization(
            pose_graph,
            o3d.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.registration.GlobalOptimizationConvergenceCriteria(),
            option)

        for point_id in range(len(pcds)):
            T_w_i = pose_graph.nodes[point_id].pose
            # print("point id {0}'s T_w_i: {1}".format(point_id, T_w_i))
            # pcds[point_id].transform(pose_graph.nodes[point_id].pose)  #pose_graph.nodes[point_id].pose = Twi

        return T_w_i, pose_graph

    def visualize_combined_point_clouds(self, pcds, pose_graph):
        # print("Transform points and display")
        pcd_combined = o3d.geometry.PointCloud()
        for point_id in range(len(pcds)):
            pcds[point_id].transform(pose_graph.nodes[point_id].pose)
            pcd_combined += pcds[point_id]
        pcd_combined_down = o3d.geometry.voxel_down_sample(pcd_combined, voxel_size=self.voxel_size)
        # o3d.io.write_point_cloud("multiway_registration.ply", pcd_combined_down)
        o3d.visualization.draw_geometries([pcd_combined])
