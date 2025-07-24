import open3d as o3d
import potpourri3d as pp3d
import numpy as np
import trimesh
import copy
import os

def load_obj_as_mesh(filename):
    if filename.endswith(".ply"):
        # 点云文件
        pcd = o3d.io.read_point_cloud(filename)
        V = np.asarray(pcd.points)
        F = np.zeros((0, 3), dtype=np.int32)
        return V, F, pcd
    else:
        tm = trimesh.load(filename, process=True)
        if not isinstance(tm, trimesh.Trimesh):
            raise ValueError("仅支持三角面片 obj")
        V = np.array(tm.vertices)
        F = np.array(tm.faces)
        return V, F, tm

def pick_vertex_indices_from_pointcloud(points):
    print("请选择多个点（Shift+左键）用于路径或方向定义，按 Q 退出")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
    picked_ids = vis.get_picked_points()
    if len(picked_ids) < 2:
        raise RuntimeError("至少需要选择两个点")
    print(f"选中的顶点索引: {picked_ids}")
    return picked_ids

def fit_arc_and_sample_points(path_pts, num_samples=30):
    if len(path_pts) < 3:
        raise ValueError("至少需要三个点来拟合圆弧")
    centroid = path_pts.mean(axis=0)
    pts_centered = path_pts - centroid
    U, S, Vt = np.linalg.svd(pts_centered)
    normal = Vt[2]
    plane_basis = Vt[:2]
    pts_2d = pts_centered @ plane_basis.T
    A = np.hstack([2 * pts_2d, np.ones((len(pts_2d), 1))])
    b = np.sum(pts_2d ** 2, axis=1).reshape(-1, 1)
    x = np.linalg.lstsq(A, b, rcond=None)[0].flatten()
    cx, cy, c = x
    center_2d = np.array([cx, cy])
    radius = np.sqrt(c + cx ** 2 + cy ** 2)
    angles = np.arctan2(pts_2d[:, 1] - cy, pts_2d[:, 0] - cx)
    angles = np.unwrap(angles)
    start_angle = angles[0]
    end_angle = angles[-1]
    sampled_angles = np.linspace(start_angle, end_angle, num_samples)
    sampled_pts_2d = np.stack([
        cx + radius * np.cos(sampled_angles),
        cy + radius * np.sin(sampled_angles)
    ], axis=1)
    sampled_pts_3d = sampled_pts_2d @ plane_basis + centroid
    return sampled_pts_3d

def visualize_path(V, F, path_pts):
    geometries = []
    if F is not None and len(F) > 0:
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(V)
        mesh.triangles = o3d.utility.Vector3iVector(F)
        mesh.compute_vertex_normals()
        mesh.paint_uniform_color([0.7, 0.7, 0.7])
        geometries.append(mesh)
    else:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(V)
        pcd.paint_uniform_color([0.7, 0.7, 0.7])
        geometries.append(pcd)
    path_line = o3d.geometry.LineSet()
    path_line.points = o3d.utility.Vector3dVector(path_pts)
    path_line.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(path_pts) - 1)])
    path_line.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(path_pts) - 1)])
    geometries.append(path_line)
    o3d.visualization.draw_geometries(geometries)

def compute_directions(path_pts):
    directions = path_pts[1:] - path_pts[:-1]
    norms = np.linalg.norm(directions, axis=1, keepdims=True)
    directions = directions / (norms + 1e-8)
    directions = np.vstack([directions, directions[-1]])
    return directions

def rotation_from_vector_to_vector(v1, v2):
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    axis = np.cross(v1, v2)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-8:
        return np.eye(3)
    axis = axis / axis_norm
    angle = np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    return R

def save_model_along_path_as_objs_custom(mesh, path_pts, mesh_direction_pts, output_folder="saved_objs"):
    os.makedirs(output_folder, exist_ok=True)
    original_mesh = copy.deepcopy(mesh)
    v1 = np.asarray(mesh_direction_pts[0])
    v2 = np.asarray(mesh_direction_pts[1])
    model_forward = v2 - v1
    model_forward /= np.linalg.norm(model_forward)
    mesh_center = np.asarray(mesh_direction_pts[2])
    directions = compute_directions(path_pts)

    for i, (pos, dir_vec) in enumerate(zip(path_pts, directions)):
        tmp_mesh = copy.deepcopy(original_mesh)
        if i == 0:
            filename = os.path.join(output_folder, f"frame_{i:04d}.obj")
            o3d.io.write_triangle_mesh(filename, tmp_mesh)
            print(f"Saved {filename}")
            continue
        R = rotation_from_vector_to_vector(model_forward, dir_vec)
        tmp_mesh.rotate(R, center=mesh_center)
        tmp_mesh.translate(pos - mesh_center, relative=True)
        filename = os.path.join(output_folder, f"frame_{i:04d}.obj")
        o3d.io.write_triangle_mesh(filename, tmp_mesh)
        print(f"Saved {filename}")

if __name__ == "__main__":
    # === 1. 加载路径点云（或 mesh） ===
    path_obj_path = "D:\\hand_eye_calibration\\EyeInHand_calibration\\brushleap\\mergeed.ply"
    V, F, _ = load_obj_as_mesh(path_obj_path)

    print("请在路径点云中选择多个点（Shift+左键，至少3个）用于拟合弧线")
    vertex_ids = pick_vertex_indices_from_pointcloud(V)
    raw_path_pts = np.array([V[i] for i in vertex_ids])
    path_pts = fit_arc_and_sample_points(raw_path_pts, num_samples=30)
    visualize_path(V, F, path_pts)

    # === 2. 加载刷子模型 ===
    mesh_path = "D:\\hand_eye_calibration\\EyeInHand_calibration\\brushleap\\output_with_facesbrushsimple.obj"
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0.8, 0.8, 0.8])

    print("请选择刷子模型上的三个点（前两个定义方向，第三个为旋转中心）")
    mesh_direction_ids = pick_vertex_indices_from_pointcloud(np.asarray(mesh.vertices))
    if len(mesh_direction_ids) != 3:
        raise RuntimeError("必须选择三个点（前两个定义方向，第三个为旋转中心）")
    mesh_direction_pts = [np.asarray(mesh.vertices)[i] for i in mesh_direction_ids]

    # === 3. 沿路径保存刷子模型 ===
    save_model_along_path_as_objs_custom(mesh, path_pts, mesh_direction_pts, output_folder="saved_objs")
