import open3d as o3d
import numpy as np
voxel_size = 0.01
max_iteration_num = 2000

#pcd files in filenames have been pre-processed to remove the floor and other surfaces that are too far away.
filenames = ["Scan0000/cloud.pcd", "Scan0001/cloud.pcd", "Scan0002/cloud.pcd", "Scan0003/cloud.pcd", "Scan0004/cloud.pcd", "Scan0005/cloud.pcd", "Scan0006/cloud.pcd", "Scan0007/cloud.pcd", "Scan0008/cloud.pcd", "Scan0009/cloud.pcd"]
folder = "/home/wangweihan/PycharmProjects/MergePointClouds/Scans_Taken_By_Robot/"

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
    return inlier_cloud

#read camera pose, this camera pose will be ICP's initial value, and then optimize this value.
def load_poses():
    filenames = ["Scan0000/CameraPose", "Scan0001/CameraPose", "Scan0002/CameraPose", "Scan0003/CameraPose",
                 "Scan0004/CameraPose", "Scan0005/CameraPose", "Scan0006/CameraPose", "Scan0007/CameraPose",
                 "Scan0008/CameraPose", "Scan0009/CameraPose"]
    local_folder = "/home/wangweihan/PycharmProjects/MergePointClouds/Scans_Taken_By_Robot/"
    poses = [np.identity(4) for _ in range(10)]
    print(np.identity(4))
    for i in range(10):
        path = local_folder + filenames[i]
        input = np.loadtxt(path, delimiter=' ')
        poses[i] = input
        # print(input)

    return poses

def load_point_clouds():
    pcds = []

    for i in range(0, 10):

        path = folder + filenames[i]
        pcd = o3d.io.read_point_cloud(path)
        # cl, ind = pcd.remove_statistical_outlier(nb_neighbors=1000, std_ratio=2.0)
        # pcd = find_inlier_outlier(pcd, ind)
        pcds.append(pcd)

    return pcds

def pairwise_registration(source, target, icp_coarse):
    print("Apply point-to-plane ICP")
    # voxel_size = 0.05
    max_correspondence_distance_coarse = voxel_size * 15
    max_correspondence_distance_fine = voxel_size * 1.5
    source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))

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


def full_registration(pcds, poses, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            #icp_coarse is transformation from source camera frame to target camer frame
            icp_coarse = np.linalg.inv(poses[target_id]).dot(poses[source_id])  #icp_coarse = T_camera(i+1)_base * T_base_camera(i) = T_camera(i+1)_camera(i)
            transformation_icp, information_icp = pairwise_registration(pcds[source_id], pcds[target_id], icp_coarse) #ICP => T_target_source
            print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry) #odometry = Tiw

                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(odometry)))   #optimize this Ti, Ti is a pose matrix wich transform Pi into to global space Ti = Twi = T0i
                pose_graph.edges.append(            #inv(odometry) = Ti
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
    return pose_graph


def main():
    pcds = load_point_clouds()
    poses = load_poses()
    print("Full registration ...")
    max_correspondence_distance_coarse = voxel_size * 15
    max_correspondence_distance_fine = voxel_size * 1.5
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        pose_graph = full_registration(pcds, poses,
                                       max_correspondence_distance_coarse,
                                       max_correspondence_distance_fine)

    print("Optimizing PoseGraph ...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine,
        edge_prune_threshold=0.25,
        reference_node=0)  #This is the node id that is cosidered to be the global space

    criteria = o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria()
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        criteria, option)

    print("Transform points and display")


    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(pcds)):
        pcds[point_id].transform(pose_graph.nodes[point_id].pose)
        pcd_combined += pcds[point_id]

    #show complete dense taskboard
    o3d.io.write_point_cloud("complete_model.pcd", pcd_combined)
    o3d.visualization.draw_geometries([pcd_combined])

if __name__ == '__main__':
    main()