#pragma once

#include <open3d/Open3D.h>
#include <memory>
#include <vector>
#include <string>

// Forward declaration
class CameraInterface;

class Scanner {
public:
    Scanner(std::shared_ptr<CameraInterface> camera_interface);
    
    // 扫描牙模并返回点云
    std::shared_ptr<open3d::geometry::PointCloud> scan_model();
    
    // 从多视角图像重建点云
    std::shared_ptr<open3d::geometry::PointCloud> reconstruct_pointcloud(
        const std::vector<std::shared_ptr<open3d::geometry::Image>>& images);
    
    // 计算牙模与末端坐标关系
    struct PoseRelation {
        std::vector<double> translation;
        std::vector<double> rotation;
    };
    PoseRelation compute_pose_relation(
        const std::shared_ptr<open3d::geometry::PointCloud>& pointcloud,
        const std::vector<double>& calibration_data);

private:
    std::shared_ptr<CameraInterface> camera_;
}; 