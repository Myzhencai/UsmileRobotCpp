#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <filesystem>
#include <algorithm>
#include "ArmController.h"
#include "IcpFusion.hpp"
#include <pcl/io/ply_io.h>

namespace fs = std::filesystem;

int main() {
    std::string folder = "./ply_folder"; // TODO: 替换为你的ply文件夹路径
    std::vector<fs::path> ply_files;
    for (const auto& entry : fs::directory_iterator(folder)) {
        if (entry.path().extension() == ".ply") {
            ply_files.push_back(entry.path());
        }
    }
    if (ply_files.empty()) {
        std::cerr << "No ply files found in folder: " << folder << std::endl;
        return 1;
    }
    // 按自然数顺序排序（如1,2,3,...,10）
    std::sort(ply_files.begin(), ply_files.end(), [](const fs::path& a, const fs::path& b) {
        auto get_number = [](const fs::path& p) -> int {
            std::string stem = p.stem().string();
            try {
                return std::stoi(stem);
            } catch (...) {
                return 0; // 非数字文件名排前面
            }
        };
        return get_number(a) < get_number(b);
    });

    // 依次加载并配准
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile<pcl::PointXYZ>(ply_files[0].string(), *result_cloud);
    for (size_t i = 1; i < ply_files.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr next_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPLYFile<pcl::PointXYZ>(ply_files[i].string(), *next_cloud);
        // ICP配准 result_cloud 和 next_cloud
        result_cloud = icppair(result_cloud, next_cloud, 0.8f, false); // voxel_size可调
    }
    std::cout << "Final merged cloud size: " << result_cloud->size() << std::endl;

    // 生成一个简单的关节轨迹（这里只做演示，实际应根据点云生成轨迹）
    c2::MovJointSegments segments;
    c2::Point p;
    p.type = c2::PointType::Joint;
    for (int i = 0; i < 6; ++i) p.apos.jntPos[i] = 0.0; // home位置
    c2::Speed speed; speed.jnt = {10,10,10,10,10,10};
    c2::Acc acc; acc.jnt = {10,10,10,10,10,10};
    c2::Zone z; z.per = 50; z.dis = 60;
    segments.AddMovJ(p, speed, acc, c2::ZoneType::Relative, z);

    // 调用ArmController运行轨迹
    ArmController arm;
    if (!arm.isconnected()) {
        std::cerr << "Arm not connected!" << std::endl;
        return 1;
    }
    bool ok = arm.StartBrushTeethTrajectoryJoint(segments);
    if (!ok) {
        std::cerr << "Trajectory execution failed!" << std::endl;
        return 1;
    }
    std::cout << "Trajectory executed successfully!" << std::endl;
    return 0;
} 