#include "TrajectoryPlanner.hpp"
// 当前机械臂没有在api上实现tools的补偿需要自己修改，这些地方都还没有用到要在力控中用到，把目标位置转换到机械臂末端的位置即可
#include <Eigen/Dense>

TrajectoryPlanner::TrajectoryPlanner(
    const std::vector<std::array<float, 3>>& topo_pointcloud, 
    const std::array<float, 6>& brush_center_pose, // [x, y, z, rx, ry, rz] 基于机械臂底座
    const Eigen::Matrix4f& tool_to_flange_transform // 机械臂home状态下，刷毛中心到机械臂末端的4x4变换矩阵
)
    : pcd_(topo_pointcloud), 
      brush_center_pose_(brush_center_pose), 
      tool_to_flange_transform_(tool_to_flange_transform) 
{
}

std::vector<std::array<float, 3>> TrajectoryPlanner::planBrushingPath(
    const std::string& mode, 
    const std::string& frequency, 
    int duration,
    int referencecode // 新增参数：指定参考轨迹编号
) {
    std::vector<std::array<float, 3>> path;

    if (mode == "reference") {
        // 基于人手采集的参考轨迹生成刷牙轨迹
        // frequency为刷牙次数，duration为总时长
        // reference_trajectories_为私有成员变量，包含多个参考轨迹
        int repeat = 1;
        if (frequency == "低") repeat = 1;
        else if (frequency == "中") repeat = 2;
        else if (frequency == "高") repeat = 3;
        // 计算每次刷牙的时长
        int single_duration = duration / repeat;

        // 选择指定编号的参考轨迹
        if (referencecode >= 0 && referencecode < static_cast<int>(reference_trajectories_.size())) {
            const auto& selected_traj = reference_trajectories_[referencecode];

            // 1. 计算base_top_toothcloud_和pcd_的质心偏移
            Eigen::Vector3f base_centroid(0, 0, 0), target_centroid(0, 0, 0);
            for (const auto& pt : base_top_toothcloud_) {
                base_centroid += Eigen::Vector3f(pt[0], pt[1], pt[2]);
            }
            if (!base_top_toothcloud_.empty())
                base_centroid /= static_cast<float>(base_top_toothcloud_.size());

            for (const auto& pt : pcd_) {
                target_centroid += Eigen::Vector3f(pt[0], pt[1], pt[2]);
            }
            if (!pcd_.empty())
                target_centroid /= static_cast<float>(pcd_.size());

            Eigen::Vector3f translation = target_centroid - base_centroid;

            // 2. 计算base_brush_pose_和brush_center_pose_的旋转差异（只考虑欧拉角的旋转部分）
            // base_brush_pose_和brush_center_pose_均为[x, y, z, rx, ry, rz]
            Eigen::Vector3f base_rpy(base_brush_pose_[3], base_brush_pose_[4], base_brush_pose_[5]);
            Eigen::Vector3f target_rpy(brush_center_pose_[3], brush_center_pose_[4], brush_center_pose_[5]);

            // 将欧拉角转换为旋转矩阵
            Eigen::Matrix3f base_rot, target_rot;
            base_rot = (
                Eigen::AngleAxisf(base_rpy[2], Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(base_rpy[1], Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(base_rpy[0], Eigen::Vector3f::UnitX())
            ).toRotationMatrix();
            target_rot = (
                Eigen::AngleAxisf(target_rpy[2], Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(target_rpy[1], Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(target_rpy[0], Eigen::Vector3f::UnitX())
            ).toRotationMatrix();

            // 旋转补偿矩阵
            Eigen::Matrix3f rot_comp = target_rot * base_rot.inverse();

            for (int i = 0; i < repeat; ++i) {
                for (const auto& pt : selected_traj) {
                    // 先减去base_centroid，旋转，再加上target_centroid
                    Eigen::Vector3f p(pt[0], pt[1], pt[2]);
                    Eigen::Vector3f p_centered = p - base_centroid;
                    Eigen::Vector3f p_rotated = rot_comp * p_centered;
                    Eigen::Vector3f p_transformed = p_rotated + target_centroid;

                    std::array<float, 3> transformed_pt = {p_transformed[0], p_transformed[1], p_transformed[2]};
                    auto adjusted_pt = applyOffset(transformed_pt);
                    path.push_back(adjusted_pt);
                }
            }
        } else {
            // 编号非法，返回空路径或可加日志
        }
        // 可以根据single_duration调整采样点密度或插值，暂略
    } else if (mode == "auto") {
        // 基于单颗牙齿点云模型自动生成轨迹（重拓扑）
        // frequency为刷牙次数，duration为总时长
        int repeat = 1;
        if (frequency == "低") repeat = 1;
        else if (frequency == "中") repeat = 2;
        else if (frequency == "高") repeat = 3;
        // 这里假设pcd_为单颗牙齿的点云
        // 伪代码：重拓扑生成轨迹（实际应实现重拓扑算法）
        std::vector<std::array<float, 3>> topo_path = pcd_; // TODO: 替换为重拓扑算法生成的轨迹

        for (int i = 0; i < repeat; ++i) {
            for (const auto& pt : topo_path) {
                auto adjusted_pt = applyOffset(pt);
                path.push_back(adjusted_pt);
            }
        }
        // 可以根据duration调整采样点密度或插值，暂略
    } else {
        // 未知模式，返回空
    }

    return path;
}

std::vector<std::array<float, 3>> TrajectoryPlanner::extractRegionPoints(const std::vector<int>& region_ids) {
    // 假设有一个成员变量 std::vector<std::vector<std::array<float, 3>>> tooth_pointclouds_;
    // 每个元素为一颗牙齿的点云，顺序与编码一致
    std::vector<std::array<float, 3>> region_points;

    for (int id : region_ids) {
        if (id >= 0 && id < static_cast<int>(tooth_pointclouds_.size())) {
            const auto& tooth_points = tooth_pointclouds_[id];
            region_points.insert(region_points.end(), tooth_points.begin(), tooth_points.end());
        }
        // else: 编号非法可加日志或忽略
    }
    return region_points;
}

std::array<float, 3> TrajectoryPlanner::applyOffset(const std::array<float, 3>& point) {
    // 根据牙刷末端工具的几何偏移做转换
    return {point[0], point[1], point[2] + offset_};
} 