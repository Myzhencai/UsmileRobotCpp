import numpy as np
import open3d as o3d
import copy


def pick_points(pcd, window_name="Pick Points"):
    print(f"\nPick points in window: {window_name}")
    print("1) Please pick points using [shift + left click]")
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=window_name)
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
    picked_ids = vis.get_picked_points()
    print("Picked indices:", picked_ids)
    return picked_ids


def pick_points_thread(pcd, window_name, save_path, result_holder):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=window_name)
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
    picked_ids = vis.get_picked_points()
    print(f"{window_name} Picked indices:", picked_ids)
    picked_pts = np.asarray(pcd.points)[picked_ids]
    np.savetxt(save_path, picked_pts)
    print(f"{window_name} 已保存点到 {save_path}")
    result_holder.extend(picked_ids)


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])  # 源点云为黄色
    target_temp.paint_uniform_color([0, 0.651, 0.929])  # 目标点云为蓝色
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp], window_name="ICP Result")


def register_via_correspondences(source, target, source_points, target_points):
    corr = np.zeros((len(source_points), 2), dtype=int)
    corr[:, 0] = np.arange(len(source_points))
    corr[:, 1] = np.arange(len(target_points))
    print("Compute a rough transform using the correspondences given by user")
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source_points, target_points, o3d.utility.Vector2iVector(corr))

    print("Perform point-to-point ICP refinement")
    threshold = 3
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())

    print("Transformation Matrix:")
    print(reg_p2p.transformation)
    draw_registration_result(source, target, reg_p2p.transformation)
    return reg_p2p.transformation


if __name__ == "__main__":
    # 加载点云
    pcd_path1 = "D:/hand_eye_calibration/robotply/filtered_pointcloud1_trans.ply"
    pcd_path2 = "D:/hand_eye_calibration/robotply/filtered_pointcloud4_trans.ply"
    pcd1 = o3d.io.read_point_cloud(pcd_path1)
    pcd2 = o3d.io.read_point_cloud(pcd_path2)

    # 对点云进行统计离群点滤波处理
    pcd1, ind1 = pcd1.remove_statistical_outlier(nb_neighbors=16, std_ratio=2.0)
    pcd2, ind2 = pcd2.remove_statistical_outlier(nb_neighbors=16, std_ratio=2.0)

    # 只为前1000个点设置颜色，其余保持原色
    def color_partial(pcd, color, n=1000):
        points = np.asarray(pcd.points)
        N = len(points)
        if pcd.has_colors():
            colors = np.asarray(pcd.colors)
        else:
            colors = np.ones((N, 3)) * 0.7  # 默认灰色
        colors[:min(n, N)] = color
        pcd.colors = o3d.utility.Vector3dVector(colors)
        return pcd

    # 例：pcd1前1000个点为红色，pcd2前1000个点为蓝色
    color_partial(pcd1, [1, 0, 0], n=10000)
    color_partial(pcd2, [0, 0, 1], n=10000)

    # 平移pcd2
    center1 = pcd1.get_center()
    center2 = pcd2.get_center()
    shift_to_align = center1 - center2
    pcd2_shifted = o3d.geometry.PointCloud(pcd2)
    pcd2_shifted.translate(shift_to_align, relative=True)
    pcd2_shifted.translate([100, 0, 0], relative=True)

    # 融合显示
    fused = pcd1 + pcd2_shifted
    # o3d.visualization.draw_geometries([fused], window_name="Partial Color Highlight")

    # 选点
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Pick Correspondences")
    vis.add_geometry(fused)
    vis.run()
    vis.destroy_window()
    picked_ids = vis.get_picked_points()
    print("Picked indices:", picked_ids)

    # 假设你每次都严格一对一选点（先选pcd1的点，再选pcd2的点，依次循环）
    N = len(picked_ids) // 2
    ids1 = picked_ids[:N]
    ids2 = picked_ids[N:]

    points = np.asarray(fused.points)
    source_points = points[ids1]
    # 还原pcd2原始坐标：先减去X轴平移，再减去中心对齐平移
    target_points = points[ids2] - np.array([100, 0, 0]) - shift_to_align

    # 保存
    with open("picked_points_source.txt", "w") as f:
        f.write("# pcd1\n")
        np.savetxt(f, source_points)
    with open("picked_points_target.txt", "w") as f:
        f.write("# pcd2\n")
        np.savetxt(f, target_points)

    # 用选点做初始配准和ICP
    src_corr = o3d.geometry.PointCloud()
    tgt_corr = o3d.geometry.PointCloud()
    src_corr.points = o3d.utility.Vector3dVector(source_points)
    tgt_corr.points = o3d.utility.Vector3dVector(target_points)

    corr = np.zeros((N, 2), dtype=int)
    corr[:, 0] = np.arange(N)
    corr[:, 1] = np.arange(N)
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(src_corr, tgt_corr, o3d.utility.Vector2iVector(corr))

    threshold = 3
    reg_p2p = o3d.pipelines.registration.registration_icp(
        pcd1, pcd2, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    print("ICP refined transformation:\n", reg_p2p.transformation)

    # 显示配准结果
    pcd1_aligned = copy.deepcopy(pcd1)
    pcd1_aligned.transform(reg_p2p.transformation)
    # o3d.visualization.draw_geometries([pcd1_aligned, pcd2])

    # 对pcd1_aligned和pcd2再做一次ICP精配准
    print("Running ICP refinement on aligned point clouds...")
    threshold_refine = 1.0  # 可根据点云密度调整
    reg_refine = o3d.pipelines.registration.registration_icp(
        pcd1_aligned, pcd2, threshold_refine, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    print("Refined transformation (after initial alignment):\n", reg_refine.transformation)
    pcd1_aligned_refined = copy.deepcopy(pcd1_aligned)
    pcd1_aligned_refined.transform(reg_refine.transformation)

    # 融合显示
    fused = pcd1_aligned_refined + pcd2
    o3d.visualization.draw_geometries([fused], window_name="Fused Point Cloud (After ICP Refinement)")

