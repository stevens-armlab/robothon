import open3d as o3d
import numpy as np

voxel_size = 0.02 #this is optional

def load_complete_model():
    pcds = []
    complete_model_path = "../point_clouds/source.pcd"
    pcd = o3d.io.read_point_cloud(complete_model_path)
    # pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
    pcds.append(pcd)
    return pcds

def load_point_clouds(pcds, voxel_size, path):
    pcd = o3d.io.read_point_cloud(path)
    # pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
    pcds.append(pcd)

    return pcds

def pairwise_registration(source, target):
    print("Apply point-to-plane ICP")
    max_correspondence_distance_coarse = voxel_size * 15
    max_correspondence_distance_fine = voxel_size * 1.5

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

def full_registration(pcds, max_correspondence_distance_coarse, max_correspondence_distance_fine):
    pose_graph = o3d.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id])
            print("Build o3d.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.registration.PoseGraphNode(
                        np.linalg.inv(odometry)))  # odometry^-1 = Ti that is a pose matrix wich transform Pi into to global space Ti = Twi = T0i
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

def estimate(pcds):
    print("Full registration starts")
    max_correspondence_distance_coarse = voxel_size * 15
    max_correspondence_distance_fine = voxel_size * 1.5

    pose_graph = full_registration(pcds,max_correspondence_distance_coarse, max_correspondence_distance_fine)
    print("Optimizing PoseGraph by nonlinear optimization starts")
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
        print("point id {0}'s T_w_i: {1}".format(point_id, T_w_i))
        # pcds[point_id].transform(pose_graph.nodes[point_id].pose)  #pose_graph.nodes[point_id].pose = Twi


    return T_w_i, pose_graph


def visualize_combined_point_clouds(pcds, pose_graph):
    print("Transform points and display")
    pcd_combined = o3d.geometry.PointCloud()
    # for point_id in range(len(pcds)):
    #     pcds[point_id].transform(pose_graph.nodes[point_id].pose)
    #     pcd_combined += pcds[point_id]
    pcds[1].transform(pose_graph.nodes[1].pose)
    pcd_combined += pcds[1]
    pcd_combined_down = o3d.geometry.voxel_down_sample(pcd_combined, voxel_size=voxel_size)
    # o3d.io.write_point_cloud("multiway_registration.ply", pcd_combined_down)
    o3d.visualization.draw_geometries([pcd_combined])


if __name__ == '__main__':
    folder = "../point_clouds/"
    filenames = "target1.pcd"
    path = folder+filenames
    pcd_model = load_complete_model()
    pcds = load_point_clouds(pcd_model, voxel_size, path)

    #Estimate transformation(T_mr_c)
    Twi, pose_graph = estimate(pcds)
    print("T_mr_c: {0}".format(Twi))

    #visualize and save pcd file
    visualize_combined_point_clouds(pcds, pose_graph)
    print("Finish!")