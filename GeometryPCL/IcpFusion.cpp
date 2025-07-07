#include "icpFusion.hpp"


using namespace std;

void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_normals)
{
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    n.setNumberOfThreads(8);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(50);
    n.compute(*normals);
    pcl::concatenateFields(*cloud, *normals, *cloud_normals);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr icppair(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target,
    float voxel_size,
    bool visualize)
{
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

    pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    cloud_with_normal(source, source_with_normals);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    cloud_with_normal(target, target_with_normals);

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

    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source, *out_cloud, transformation);

    if (visualize)
    {
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
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::microseconds(10000));
        }
    }
    return out_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr icppair_with_init(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr target,
    const Eigen::Matrix4f &init_transform,
    float voxel_size,
    bool visualize)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_init(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source, *source_init, init_transform);

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

    pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    cloud_with_normal(source_init, source_with_normals);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    cloud_with_normal(target, target_with_normals);

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

    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source_init, *out_cloud, transformation);

    if (visualize)
    {
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
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::microseconds(10000));
        }
    }
    return out_cloud;
}

void pointcloud_to_mesh(PointCloudT::Ptr cloud, const std::string &save_path, float voxel_size)
{
    if (!cloud || cloud->empty())
    {
        std::cerr << "Input cloud is empty, cannot reconstruct mesh." << std::endl;
        return;
    }
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    ne.setNumberOfThreads(8);
    pcl::search::KdTree<PointT>::Ptr tree_xyz(new pcl::search::KdTree<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree_xyz);
    ne.setKSearch(20);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
    ne.compute(*normals);
    for (size_t i = 0; i < normals->size(); ++i) {
        Eigen::Vector3f normal(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
        Eigen::Vector3f to_centroid = centroid.head<3>() - cloud->points[i].getVector3fMap();
        if (normal.dot(to_centroid) > 0) {
            normals->points[i].normal_x *= -1;
            normals->points[i].normal_y *= -1;
            normals->points[i].normal_z *= -1;
        }
    }
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    pcl::PolygonMesh mesh;
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    gp3.setSearchRadius(voxel_size * 3.0f);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4);
    gp3.setMinimumAngle(M_PI / 18);
    gp3.setMaximumAngle(2 * M_PI / 3);
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree);
    gp3.reconstruct(mesh);
    pcl::io::savePLYFile(save_path, mesh);
    std::cout << "Saved mesh to: " << save_path << std::endl;
}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile<pcl::PointXYZ>("D:/Open3DDemo/ResultData/test6/1.ply", *source);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile<pcl::PointXYZ>("D:/Open3DDemo/ResultData/test6/2.ply", *target);
    Eigen::Matrix4f init_transform;
    init_transform << 0.947263, 0.294074, -0.127332, -15.6769,
        -0.288639, 0.955579, 0.059639, -3.83944,
        0.139214, -0.0197408, 0.990066, -13.6697,
        0, 0, 0, 1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned = icppair_with_init(source, target, init_transform, 0.06f, true);
    pointcloud_to_mesh(aligned, "D:/Open3DDemo/ResultData/test6/aligned_result_mesh.ply", 0.06f);
    std::cout << "Saved aligned point cloud: D:/Open3DDemo/ResultData/test6/aligned_result.ply" << std::endl;
    return 0;
}




