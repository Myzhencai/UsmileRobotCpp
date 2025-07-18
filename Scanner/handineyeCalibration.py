import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d



# 标定板参数（直接赋值，不再读取yaml）
XX = 7  # 列数
YY = 11 # 行数
L_horizontal = 4  # 水平间距，单位米
L_vertical = 2    # 垂直间距，单位米


def find_corners_v2(img_gray, w=7, h=11):
    """
    更鲁棒的非对称圆点阵列检测，适合白色圆点，参数可调。
    """
    # 亚像素角点精确化的终止准则
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 300, 0.0001)
    # 创建 Blob 检测器参数
    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = True
    params.blobColor = 255  # 检测白色圆点
    params.minArea = 10
    params.maxArea = 5000
    params.filterByCircularity = True
    params.minCircularity = 0.7
    params.minDistBetweenBlobs = 5
    # 创建检测器
    detector = cv2.SimpleBlobDetector_create(params)
    # 使用非对称圆点阵列检测
    ret, centers = cv2.findCirclesGrid(
        img_gray, (w, h),
        flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
        blobDetector=detector
    )
    if ret:
        # 亚像素精细化
        cv2.cornerSubPix(img_gray, centers, (5, 5), (-1, -1), criteria)
        return centers
    else:
        print("未检测到圆点！")
        return None


def detect_circle_grid_with_roi_v2(image_path, w=7, h=11, show_result=True):
    img = cv2.imread(image_path)
    if img is None:
        print(f"无法读取图片: {image_path}")
        return None

    # 1. 显示缩小4倍的图片让用户选ROI
    img_small = cv2.resize(img, (img.shape[1]//4, img.shape[0]//4), interpolation=cv2.INTER_AREA)
    print("请在缩小后的图片上框选棋盘格区域，回车确认，ESC取消")
    roi_small = cv2.selectROI("Select ROI (缩小4倍)", img_small, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow("Select ROI (缩小4倍)")
    x_s, y_s, w_s, h_s = roi_small
    if w_s == 0 or h_s == 0:
        print("未选择区域，退出")
        return None

    # 2. ROI坐标映射回原图
    x = x_s * 4
    y = y_s * 4
    w_roi = w_s * 4
    h_roi = h_s * 4

    img_roi = img[y:y+h_roi, x:x+w_roi]
    gray = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)

    # 3. 先做二值化
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    cv2.imshow("binary Result", binary)
    cv2.waitKey(0)

    # 4. 调用 find_corners_v2 检测
    corners = find_corners_v2(binary, w=w, h=h)

    # 5. 检测到的圆点坐标加上ROI偏移
    if corners is not None:
        corners[:, 0, 0] += x
        corners[:, 0, 1] += y

    # 6. 显示结果
    if show_result:
        img_show = img.copy()
        if corners is not None:
            cv2.drawChessboardCorners(img_show, (w, h), corners, True)
            for idx, pt in enumerate(corners, 1):
                px, py = int(pt[0][0]), int(pt[0][1])
                cv2.putText(img_show, str(idx), (px, py-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        else:
            print("角点未找到")
        img_show_small = cv2.resize(img_show, (img_show.shape[1]//4, img_show.shape[0]//4), interpolation=cv2.INTER_AREA)
        cv2.imshow("Detection Result", img_show_small)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return corners


def plot_asymmetric_circle_grid_3d(XX=7, YY=11, L_horizontal=0.004, L_vertical=0.002):
    """
    绘制非对称7x11白点圆阵标定板的3D点，z=0，坐标系建立在标定板上，并标注点的顺序号
    XX: 列数（每行圆点数，宽方向）
    YY: 行数（每列圆点数，高方向）
    L_horizontal: 水平（宽方向）圆点间距（单位：米），如4mm
    L_vertical: 垂直（高方向）圆点间距（单位：米），如2mm
    """
    # 生成圆点的3D坐标
    objp = np.zeros((XX * YY, 3), np.float32)
    # 不对称圆点阵列的世界坐标系生成方式
    # 水平间距4mm，垂直间距2mm
    objp[:, :2] = np.array([ [((2*j) + i % 2) * L_horizontal / 2, i * L_vertical] for i in range(YY) for j in range(XX) ])
    X = objp[:,0]
    Y = objp[:,1]
    Z = objp[:,2]

    fig = plt.figure(figsize=(6,5))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(X, Y, Z, c='b', s=40)

    # 标注点的顺序号（1,2,3,...）
    for idx, (x, y, z) in enumerate(zip(X, Y, Z), 1):
        ax.text(float(x), float(y), float(z)+0.0005, str(idx), color='red', fontsize=8, ha='center', va='bottom')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Asymmetric Circle Grid 3D Points (7x11, H={L_horizontal*1000:.0f}mm, V={L_vertical*1000:.0f}mm)')
    # 只显示z=0平面
    ax.set_box_aspect([1, 0.8, 0.2])
    ax.set_zlim(-0.001, 0.001)
    plt.show()


def handeye_calibration_from_folder(
    image_dir,
    pose_list,
    mtx,
    dist,
    objp,
    w=7,
    h=11,
    show_result=False
):
    import os
    import re
    # 获取图片文件名并排序
    files = [f for f in os.listdir(image_dir) if f.endswith('.bmp')]
    def extract_number(filename):
        match = re.search(r'(\d+)', filename)
        return int(match.group(1)) if match else -1
    files_sorted = sorted(files, key=extract_number)

    rvecs = []
    tvecs = []
    valid_poses = []

    for idx, fname in enumerate(files_sorted):
        img_path = os.path.join(image_dir, fname)
        print(f'处理图片: {img_path}')
        corners = detect_circle_grid_with_roi_v2(img_path, w=w, h=h, show_result=show_result)
        if corners is not None and corners.shape[0] == objp.shape[0]:
            imgp = corners.reshape(-1, 2).astype(np.float32)
            # PnP
            success, rvec, tvec = cv2.solvePnP(objp, imgp, mtx, dist)
            if not success:
                print(f"solvePnP失败: {fname}")
                continue
            rvecs.append(rvec)
            tvecs.append(tvec)
            valid_poses.append(pose_list[idx])
            print(f"{fname} 检测成功，加入手眼标定")
        else:
            print(f"{fname} 检测失败，跳过")
    print(f"有效样本数: {len(rvecs)}")
    return rvecs, tvecs, valid_poses


# 机械臂姿态转为R_tool, t_tool
def euler_angles_to_rotation_matrix(rx, ry, rz):
    rx = np.deg2rad(rx)
    ry = np.deg2rad(ry)
    rz = np.deg2rad(rz)
    Rx = np.array([[1, 0, 0],
                    [0, np.cos(rx), -np.sin(rx)],
                    [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                    [0, 1, 0],
                    [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                    [np.sin(rz), np.cos(rz), 0],
                    [0, 0, 1]])
    return Rz @ Ry @ Rx

def pose_to_homogeneous_matrix(pose):
    x, y, z, rx, ry, rz = pose
    R = euler_angles_to_rotation_matrix(rx, ry, rz)
    t = np.array([x, y, z]).reshape(3, 1)
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = t[:, 0]
    return H


def transform_and_save_pointclouds(R_, t_, pose_list, ply_dir, save_suffix="_trans", z_min=180, z_max=230):
    # 获取ply文件并排序
    ply_files = [f for f in os.listdir(ply_dir) if f.endswith('.ply')]
    ply_files_sorted = sorted(ply_files, key=lambda x: int(os.path.splitext(x)[0].split('_')[-1]) if x.split('_')[-1].split('.')[0].isdigit() else -1)
    assert len(ply_files_sorted) == len(pose_list), "点云文件数与姿态数不一致！"
    pcds = [o3d.io.read_point_cloud(os.path.join(ply_dir, f)) for f in ply_files_sorted]
    for i, (pcd, pose) in enumerate(zip(pcds, pose_list)):
        # 1. 直通滤波（z轴范围）
        points = np.asarray(pcd.points)
        mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        filtered_points = points[mask]
        filtered_pcd = o3d.geometry.PointCloud()
        filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
        if pcd.has_colors():
            colors = np.asarray(pcd.colors)
            filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
        # 2. 统计离群点滤波
        filtered_pcd, ind = filtered_pcd.remove_statistical_outlier(nb_neighbors=16, std_ratio=2.0)
        # 3. 变换到机械臂基坐标系
        H_tool = pose_to_homogeneous_matrix(pose)
        H_cam2tool = np.eye(4)
        H_cam2tool[:3, :3] = R_
        H_cam2tool[:3, 3] = t_.reshape(3)
        H_cam2base = H_tool @ H_cam2tool
        points = np.asarray(filtered_pcd.points)
        points_h = np.hstack([points, np.ones((points.shape[0], 1))])
        points_base = (H_cam2base @ points_h.T).T[:, :3]
        pcd_base = o3d.geometry.PointCloud()
        pcd_base.points = o3d.utility.Vector3dVector(points_base)
        if filtered_pcd.has_colors():
            pcd_base.colors = filtered_pcd.colors
        if not pcd_base.has_colors():
            color = [1, 0, 0] if i == 0 else [0, 1, 0]
            pcd_base.paint_uniform_color(color)
        orig_path = os.path.join(ply_dir, ply_files_sorted[i])
        folder, filename = os.path.split(orig_path)
        name, ext = os.path.splitext(filename)
        save_name = f"{name}{save_suffix}{ext}"
        save_path = os.path.join(folder, save_name)
        o3d.io.write_point_cloud(save_path, pcd_base)
        print(f"Transformed point cloud saved to: {save_path}")
    print("所有点云已滤波、变换并保存。")


def main():
    # 标定板参数
    XX = 7
    YY = 11
    L_horizontal = 4
    L_vertical = 2

    # 3D点
    objp = np.zeros((XX * YY, 3), np.float32)
    objp[:, :2] = np.array([ [((2*j) + i % 2) * L_horizontal / 2, i * L_vertical] for i in range(YY) for j in range(XX) ])

    # 相机内参
    mtx = np.array([
        [4465.71, 0, 1239.92],
        [0, 4466.26, 1008.76],
        [0,  0,  1]
    ], dtype=np.float32)
    dist = np.array([
        -0.0488621, 0.205381, -0.000102168, -0.000320252, -1.1551
    ])

    # 机械臂姿态（与图片顺序一一对应）
    pose = [
        [466.596, 269.629,310.329,-136.374,-0.056,80.547],
        [436.654, 248.387,350.418,-129.928,17.200,57.835],
        [400.142, 225.040,347.870,-148.921,-2.967,72.456],
        [202.294, 487.890,365.363,-172.109,9.550,-9.242],
        [133.939, 533.777,298.504,177.661,-11.706,-9.674],
        [125.294, 531.758,267.106,-151.705,-25.953,-41.950],
        [144.254, 531.506,261.739,-156.102,-25.946,-34.992],
        [121.606, 499.039,255.412,-147.71,-19.577,-29.965],
        [327.908, 610.077,313.361,-166.266,-6.811,-81.072],
        [394.811, 526.117,366.510,172.525,11.581,-86.571],
        [270.383, 553.787,361.658,-147.779,8.122,-78.432],
        [325.356, 448.816,382.376,-168.271,27.800,-3.853]
    ]

    

    # 路径
    image_dir = "D:/hand_eye_calibration/robotcalibratepng/img"
    # 检测与PnP
    rvecs, tvecs, valid_poses = handeye_calibration_from_folder(
        image_dir, pose, mtx, dist, objp, w=7, h=11, show_result=False
    )

    # 只用有效的机械臂姿态
    R_tool = []
    t_tool = []
    for p in valid_poses:
        H_tool = pose_to_homogeneous_matrix(p)
        R_tool.append(H_tool[0:3, 0:3])
        t_tool.append(H_tool[0:3, 3])

    # 手眼标定
    if len(rvecs) > 0:
        R_, t_ = cv2.calibrateHandEye(R_tool, t_tool, rvecs, tvecs, cv2.CALIB_HAND_EYE_TSAI)
        print('Hand-Eye Calibration Result:')
        print('Rotation:\n', R_)
        print('Translation:\n', t_)

        valid_poses =[
        [-74.357, 532.950,483.068,179.124,1.214,70.807],
        [-114.412, 510.367,466.645,145.825,9.066,96.474],
        [-89.975, 560.312,467.485,154.512,34.193,134.682],
        [12.443, 621.743,520.518,-165.667,32.310,13.631]
        ]
        # 点云批量变换与保存
        ply_dir = "D:/hand_eye_calibration/robotply/filterpointcloud"
        transform_and_save_pointclouds(R_, t_, valid_poses, ply_dir, save_suffix="_trans", z_min=180, z_max=230)
    else:
        print('没有足够的有效样本进行手眼标定。')

if __name__ == '__main__':
    main()


