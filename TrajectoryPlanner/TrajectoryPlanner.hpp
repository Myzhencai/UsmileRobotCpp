#ifndef TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_HPP

#include <vector>
#include <array>

class TrajectoryPlanner {
public:
    // 构造函数
    TrajectoryPlanner(const std::vector<std::array<float, 3>>& pointcloud, float tool_offset);
    
    // 规划刷牙路径
    std::vector<std::array<float, 3>> planBrushingPath(int region_id, const std::string& frequency = "中", int duration = 10);

private:
    // 提取区域点云
    std::vector<std::array<float, 3>> extractRegionPoints(int region_id);
    
    // 应用工具偏移
    std::array<float, 3> applyOffset(const std::array<float, 3>& point);

    std::vector<std::array<float, 3>> pcd_;  // 点云数据
    float offset_;  // 工具偏移量
};

#endif // TRAJECTORY_PLANNER_HPP 