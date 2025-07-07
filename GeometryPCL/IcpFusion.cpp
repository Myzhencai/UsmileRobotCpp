#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/registration/icp.h> // 点到面的ICP算法

#include <pcl/visualization/pcl_visualizer.h>

#include <thread>

#include <chrono>

#include <iostream>

#include <string>

#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>

#include <pcl/registration/icp.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/time.h>

#include <vector>

#include <omp.h>

#include <pcl/filters/voxel_grid.h>

using namespace std;

typedef pcl::PointXYZ PointT;

typedef pcl::PointCloud<PointT> PointCloudT;

// 计算点云法线并将法线信息拼接到点云数据中

void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_normals)

{

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    n.setNumberOfThreads(4); // 设置并行线程数

    n.setInputCloud(cloud);

    n.setSearchMethod(tree);

    n.setKSearch(50); // 近邻点的数量

    n.compute(*normals); // 计算法线

    // 将点云数据与法线信息拼接

    pcl::concatenateFields(*cloud, *normals, *cloud_normals);
}

// 输入两个点云，返回配准后的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr icppair(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target,
    float voxel_size = 0.8f,
    bool visualize = false
) {
    // 下采样
    pcl::VoxelGrid<pcl::PointXYZ> sor_source;
    sor_source.setInputCloud(source);
    sor_source.setLeafSize(voxel_size, voxel_size, voxel_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    sor_source.filter(*source_downsampled);
    source = source_downsampled;

    pcl::VoxelGrid<pcl::PointXYZ> sor_target;
    sor_target.setInputCloud(target);
    sor_target.setLeafSize(voxel_size, voxel_size, voxel_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    sor_target.filter(*target_downsampled);
    target = target_downsampled;

    // 计算法线
    pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    cloud_with_normal(source, source_with_normals);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    cloud_with_normal(target, target_with_normals);

    // ICP
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> p_icp;
    p_icp.setInputSource(source_with_normals);
    p_icp.setInputTarget(target_with_normals);
    p_icp.setTransformationEpsilon(1e-10);
    p_icp.setMaxCorrespondenceDistance(10);
    p_icp.setEuclideanFitnessEpsilon(0.00001);
    p_icp.setMaximumIterations(135);
    pcl::PointCloud<pcl::PointNormal>::Ptr p_icp_cloud(new pcl::PointCloud<pcl::PointNormal>);
    p_icp.align(*p_icp_cloud);

    std::cout << "\nICP has converged, score is " << p_icp.getFitnessScore() << std::endl;
    Eigen::Matrix4f transformation = p_icp.getFinalTransformation();
    std::cout << "变换矩阵：\n" << transformation << std::endl;

    // 变换source点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source, *out_cloud, transformation);

    // 可视化
    if (visualize) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Registration Result"));
        int v1 = 0, v2 = 1;
        viewer->createViewPort(0, 0, 0.5, 1, v1);
        viewer->createViewPort(0.5, 0, 1, 1, v2);
        viewer->setBackgroundColor(0, 0, 0, v1);
        viewer->setBackgroundColor(0.05, 0, 0, v2);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source, 255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target, 0, 0, 255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> transe(p_icp_cloud, 255, 255, 0);
        viewer->addPointCloud(source, src_h, "source cloud", v1);
        viewer->addPointCloud(target, tgt_h, "target cloud", v1);
        viewer->addPointCloud(target, tgt_h, "target cloud1", v2);
        viewer->addPointCloud(p_icp_cloud, transe, "pcs cloud", v2);
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::microseconds(10000));
        }
    }
    return out_cloud;
}

// 输入两个点云和初始变换，返回配准后的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr icppair_with_init(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target,
    const Eigen::Matrix4f& init_transform,
    float voxel_size = 0.8f,
    bool visualize = false
) {
    // 先对source应用初始变换
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_init(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source, *source_init, init_transform);

    // 下采样
    pcl::VoxelGrid<pcl::PointXYZ> sor_source;
    sor_source.setInputCloud(source_init);
    sor_source.setLeafSize(voxel_size, voxel_size, voxel_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    sor_source.filter(*source_downsampled);
    source_init = source_downsampled;

    pcl::VoxelGrid<pcl::PointXYZ> sor_target;
    sor_target.setInputCloud(target);
    sor_target.setLeafSize(voxel_size, voxel_size, voxel_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    sor_target.filter(*target_downsampled);
    target = target_downsampled;

    // 计算法线
    pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    cloud_with_normal(source_init, source_with_normals);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    cloud_with_normal(target, target_with_normals);

    // ICP
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> p_icp;
    p_icp.setInputSource(source_with_normals);
    p_icp.setInputTarget(target_with_normals);
    p_icp.setTransformationEpsilon(1e-10);
    p_icp.setMaxCorrespondenceDistance(5);
    p_icp.setEuclideanFitnessEpsilon(0.000001);
    p_icp.setMaximumIterations(135);
    pcl::PointCloud<pcl::PointNormal>::Ptr p_icp_cloud(new pcl::PointCloud<pcl::PointNormal>);
    p_icp.align(*p_icp_cloud);

    std::cout << "\nICP has converged, score is " << p_icp.getFitnessScore() << std::endl;
    Eigen::Matrix4f transformation = p_icp.getFinalTransformation();
    std::cout << "变换矩阵：\n" << transformation << std::endl;

    // 变换source_init点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source_init, *out_cloud, transformation);

    // 可视化
    if (visualize) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Registration Result"));
        int v1 = 0, v2 = 1;
        viewer->createViewPort(0, 0, 0.5, 1, v1);
        viewer->createViewPort(0.5, 0, 1, 1, v2);
        viewer->setBackgroundColor(0, 0, 0, v1);
        viewer->setBackgroundColor(0.05, 0, 0, v2);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source_init, 255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target, 0, 0, 255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> transe(p_icp_cloud, 255, 255, 0);
        viewer->addPointCloud(source_init, src_h, "source cloud", v1);
        viewer->addPointCloud(target, tgt_h, "target cloud", v1);
        viewer->addPointCloud(target, tgt_h, "target cloud1", v2);
        viewer->addPointCloud(p_icp_cloud, transe, "pcs cloud", v2);
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::microseconds(10000));
        }
    }
    return out_cloud;
}

int main()
{
    // 加载源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile<pcl::PointXYZ>("D:/Open3DDemo/ResultData/test6/1.ply", *source);
    // 加载目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile<pcl::PointXYZ>("D:/Open3DDemo/ResultData/test6/2.ply", *target);

    // 设置初始旋转矩阵
    Eigen::Matrix4f init_transform;
    init_transform << 
        0.947263,   0.294074,  -0.127332,  -15.6769,
       -0.288639,   0.955579,   0.059639,   -3.83944,
        0.139214,  -0.0197408,  0.990066,   -13.6697,
        0,          0,          0,          1;

    // 调用icppair_with_init进行配准
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned = icppair_with_init(source, target, init_transform, 0.06f, true);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr aligned = icppair(source, target, 0.06f, true);

    // 保存配准后的点云
    // pcl::io::savePLYFileBinary("D:/Open3DDemo/ResultData/test6/aligned_result.ply", *aligned);
    std::cout << "Saved aligned point cloud: D:/Open3DDemo/ResultData/test6/aligned_result.ply" << std::endl;
    return 0;
}