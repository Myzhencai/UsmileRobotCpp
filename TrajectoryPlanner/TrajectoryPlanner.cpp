#include "TrajectoryPlanner.hpp"

TrajectoryPlanner::TrajectoryPlanner(const std::vector<std::array<float, 3>>& pointcloud, float tool_offset)
    : pcd_(pointcloud), offset_(tool_offset) {
}

std::vector<std::array<float, 3>> TrajectoryPlanner::planBrushingPath(int region_id, const std::string& frequency, int duration) {
    // 简化逻辑：生成沿牙模边缘扫过的路径点
    std::vector<std::array<float, 3>> path;
    auto region_points = extractRegionPoints(region_id);
    
    for (const auto& pt : region_points) {
        auto adjusted_pt = applyOffset(pt);
        path.push_back(adjusted_pt);
    }
    
    return path;
}

std::vector<std::array<float, 3>> TrajectoryPlanner::extractRegionPoints(int region_id) {
    // 示例函数：实际需在点云中提取对应区域
    // 这里简单返回所有点，实际应用中需要根据region_id筛选
    return pcd_;
}

std::array<float, 3> TrajectoryPlanner::applyOffset(const std::array<float, 3>& point) {
    // 根据牙刷末端工具的几何偏移做转换
    return {point[0], point[1], point[2] + offset_};
} 