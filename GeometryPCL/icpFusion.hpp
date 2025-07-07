#pragma once
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

// 计算点云法线并拼接
void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_normals);

// ICP配准
pcl::PointCloud<pcl::PointXYZ>::Ptr icppair(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target,
    float voxel_size = 0.8f,
    bool visualize = false);

// 带初始变换的ICP配准
pcl::PointCloud<pcl::PointXYZ>::Ptr icppair_with_init(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target,
    const Eigen::Matrix4f &init_transform,
    float voxel_size = 0.8f,
    bool visualize = false);

// 点云转三角网格
void pointcloud_to_mesh(PointCloudT::Ptr cloud, const std::string &save_path = "./output_mesh.ply", float voxel_size = 0.01f); 