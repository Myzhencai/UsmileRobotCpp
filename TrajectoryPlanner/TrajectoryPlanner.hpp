#ifndef TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_HPP

// 根据实际的牙齿模型和选取的区域来规划路径（坐标系全部放在机械臂底座）

#include <vector>
#include <array>

class TrajectoryPlanner {
public:
    // 构造函数
    TrajectoryPlanner(
    const std::vector<std::array<float, 3>>& topo_pointcloud, // 结构光重拓扑之后的点云
    const std::array<float, 6>& brush_center_pose, // [x, y, z, rx, ry, rz] 基于机械臂底座
    const Eigen::Matrix4f& tool_to_flange_transform); // 机械臂home状态下，刷毛中心到机械臂末端的4x4变换矩阵
    
    // 规划刷牙路径
    std::vector<std::array<float, 3>> planBrushingPath(
    const std::string& mode, 
    const std::string& frequency, 
    int duration,
    int referencecode); // 新增参数：指定参考轨迹编号

private:
    // 提取区域点云
    std::vector<std::array<float, 3>> extractRegionPoints(const std::vector<int>& region_ids)
    
    // 应用工具偏移
    std::array<float, 3> applyOffset(const std::array<float, 3>& point);

    std::vector<std::array<float, 3>> pcd_;  // 新的牙模但是拓扑过后的点云
    const std::array<float, 6>& brush_center_pose;//新的牙刷模型的pose
    float offset_;  // 工具偏移量

    // 基准牙齿点云（固定变量）
    std::vector<std::array<float, 3>> base_top_toothcloud_;
    // 基准刷毛姿态（固定变量） [x, y, z, rx, ry, rz]
    std::array<float, 6> base_brush_pose_;
    // 存储多个不同的基准参考轨迹，每个参考轨迹是一个点的数组
    std::vector<std::vector<std::array<float, 3>>> reference_trajectories_;
};

#endif // TRAJECTORY_PLANNER_HPP 