import open3d as o3d
import numpy as np

voxel_size = 0.001 #this is optional
max_iteration_num = 100

def load_complete_model():
    pcds = []
    complete_model_path = "/home/kevin/ros_ws/robothon_ws/src/robothon/vision/point_clouds/complete_model.pcd"
    pcd = o3d.io.read_point_cloud(complete_model_path)
    pcds.append(pcd)
    return pcds

def load_point_clouds(pcds, voxel_size, path):
    pcd = o3d.io.read_point_cloud(path)
    # pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
    pcds.append(pcd)

    return pcds

def pairwise_registration(source, target, icp_coarse):
    print("Apply point-to-plane ICP")
    max_correspondence_distance_coarse = voxel_size * 15
    max_correspondence_distance_fine = voxel_size * 1.5

    source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))

    # icp_coarse = o3d.pipelines.registration.registration_icp(
    #     source, target, max_correspondence_distance_coarse, np.identity(4),
    #     o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    #     o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iteration_num))
    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iteration_num))
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp

def full_registration(pcds, icp_coarse):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id], icp_coarse)
            print("Build o3d.pipelines.registration.PoseGraph")

            odometry = np.dot(transformation_icp, odometry)
            pose_graph.nodes.append(
                o3d.pipelines.registration.PoseGraphNode(
                    np.linalg.inv(odometry)))  # odometry^-1 = Ti that is a pose matrix wich transform Pi into to global space Ti = Twi = T0i
            pose_graph.edges.append(
                o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                         target_id,
                                                         transformation_icp,
                                                         information_icp,
                                                         uncertain=False))
    return pose_graph

def estimate(pcds, icp_coarse):
    print("Full registration starts")
    max_correspondence_distance_coarse = voxel_size * 15
    max_correspondence_distance_fine = voxel_size * 1.5

    pose_graph = full_registration(pcds,icp_coarse)
    print("Optimizing PoseGraph by nonlinear optimization starts")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine,
        edge_prune_threshold=0.25,
        reference_node=0)

    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)

    for point_id in range(len(pcds)):
        T_w_i = pose_graph.nodes[point_id].pose
        # print("point id {0}'s T_w_i: {1}".format(point_id, T_w_i))
        # pcds[point_id].transform(pose_graph.nodes[point_id].pose)  #pose_graph.nodes[point_id].pose = Twi


    return T_w_i, pose_graph


def visualize_combined_point_clouds(pcds, pose_graph):
    print("Transform points and display")
    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(pcds)):
        pcds[point_id].transform(pose_graph.nodes[point_id].pose)
        pcd_combined += pcds[point_id]
    # pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
    # o3d.io.write_point_cloud("multiway_registration.ply", pcd_combined_down)
    o3d.visualization.draw_geometries([pcd_combined])

def load_poses():
    filenames = ["Scan0000/CameraPose", "Scan0001/CameraPose", "Scan0002/CameraPose", "Scan0003/CameraPose",
                 "Scan0004/CameraPose", "Scan0005/CameraPose", "Scan0006/CameraPose", "Scan0007/CameraPose",
                 "Scan0008/CameraPose", "Scan0009/CameraPose"]
    local_folder = "/home/wangweihan/PycharmProjects/MergePointClouds/Scans_Taken_By_Robot/"
    poses = [np.identity(4) for _ in range(10)]
    for i in range(10):
        path = local_folder + filenames[i]
        input = np.loadtxt(path, delimiter=' ')
        poses[i] = input
        # print(input)

    return poses

if __name__ == '__main__':
    folder = "/home/wangweihan/PycharmProjects/MergePointClouds/Scans_Taken_By_Robot/"
    filenames = "Scan0004/cloud.pcd"
    path = folder+filenames
    print(o3d.__version__)
    pcd_model = load_complete_model()
    pcds = load_point_clouds(pcd_model, voxel_size, path)

    #test an example
    poses = load_poses()
    icp_coarse = np.linalg.inv(poses[4]).dot(poses[0]) #change poses[4] to current target camera pose
    print("icp_coarse: {0}".format(np.linalg.inv(icp_coarse)))
    # Estimate transformation(T_mr_c)
    Twi, pose_graph = estimate(pcds, icp_coarse)
    print("T_mr_c: {0}".format(Twi))

    #visualize and save pcd file
    visualize_combined_point_clouds(pcds, pose_graph)
    print("Finish!")