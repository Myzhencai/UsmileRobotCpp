#include "Scanner.h"
#include <iostream>

/**
 * @brief 构造函数
 */
Scanner::Scanner() : is_initialized_(false), camera_group_id_(0) {
}

/**
 * @brief 析构函数
 */
Scanner::~Scanner() {
    clear_scanned_pointclouds();
    if (is_initialized_) {
        uninitialize();
    }
}

/**
 * @brief 初始化扫描仪
 * @param mode 设备模式：0-USB设备，1-网络设备
 * @param dev_num 设备数量
 * @return 初始化是否成功
 */
bool Scanner::initialize(int mode, int dev_num) {
    // 检查是否已经初始化
    if (is_initialized_) {
        std::cout << "[警告] 扫描仪已经初始化" << std::endl;
        return true;
    }

    // 初始化SDK
    if (!TJST3DModuleInit(mode, dev_num)) {
        std::cout << "[错误] 扫描仪初始化失败" << std::endl;
        return false;
    }

    // 设置默认参数
    TJST3DSetProcessorUnit(1);  // 使用GPU进行点云处理
    TJST3DSetModuleType(0);     // 设置为双目模式
    TJST3DSetWorkMode(0);       // 设置为扫描模式

    is_initialized_ = true;
    std::cout << "[信息] 扫描仪初始化成功" << std::endl;
    return true;
}

/**
 * @brief 释放扫描仪资源
 */
void Scanner::uninitialize() {
    if (!is_initialized_) {
        return;
    }

    // 释放SDK资源
    TJST3DModuleUninit();
    is_initialized_ = false;
    std::cout << "[信息] 扫描仪资源已释放" << std::endl;
}

/**
 * @brief 扫描牙模并返回点云数据（单次扫描）
 * @return 扫描得到的点云数据
 */
sn3DCore::sn3DRangeData* Scanner::scan_model() {
    // 检查扫描仪是否已初始化
    if (!is_initialized_) {
        std::cout << "[错误] 扫描仪未初始化" << std::endl;
        return nullptr;
    }

    std::cout << "[扫描] 获取牙模点云..." << std::endl;
    
    // 创建SDK点云对象
    sn3DCore::sn3DRangeData* sdk_pointcloud = new sn3DCore::sn3DRangeData();
    
    // 执行扫描操作
    if (!TJST3DCreatePointcloudFromScan(camera_group_id_, sdk_pointcloud)) {
        std::cout << "[错误] 扫描失败" << std::endl;
        TJST3DRangeDelete(sdk_pointcloud);
        return nullptr;
    }
    
    return sdk_pointcloud;
}

/**
 * @brief 根据给定的平移和旋转对点云进行变换
 * @param pointcloud 输入点云数据
 * @param translation 平移向量 (x, y, z)
 * @param rotation 旋转欧拉角 (rx, ry, rz)
 * @return 变换后的点云数据
 */
sn3DCore::sn3DRangeData* Scanner::transform_pointcloud(
    sn3DCore::sn3DRangeData* pointcloud,
    const std::array<double, 3>& translation,
    const std::array<double, 3>& rotation) {
    std::cout << "[变换] 对点云进行位姿变换" << std::endl;
    
    // 检查输入点云是否有效
    if (!pointcloud) {
        std::cout << "[错误] 输入点云为空" << std::endl;
        return nullptr;
    }

    // 创建新的点云对象用于存储变换结果
    sn3DCore::sn3DRangeData* transformed_pointcloud = new sn3DCore::sn3DRangeData();
    
    // 构建变换矩阵
    float transform_matrix[16] = {0};
    // 设置旋转部分 (使用欧拉角构建旋转矩阵)
    // TODO: 根据rotation计算旋转矩阵并填充transform_matrix[0-8]
    
    // 设置平移部分
    transform_matrix[12] = translation[0];
    transform_matrix[13] = translation[1];
    transform_matrix[14] = translation[2];
    transform_matrix[15] = 1.0f;

    // 应用变换
    // 获取点云数据
    std::vector<Point3f>& points = pointcloud->GetPoints();
    std::vector<Point3f>& transformed_points = transformed_pointcloud->GetPoints();
    
    // 复制点云基本属性
    transformed_pointcloud->SetMap(pointcloud->GetW(), pointcloud->GetH());
    
    // 对每个点应用变换矩阵
    for (const auto& point : points) {
        Point3f transformed_point;
        // 应用旋转和平移
        transformed_point.x = point.x * transform_matrix[0] + point.y * transform_matrix[4] + point.z * transform_matrix[8] + transform_matrix[12];
        transformed_point.y = point.x * transform_matrix[1] + point.y * transform_matrix[5] + point.z * transform_matrix[9] + transform_matrix[13];
        transformed_point.z = point.x * transform_matrix[2] + point.y * transform_matrix[6] + point.z * transform_matrix[10] + transform_matrix[14];
        transformed_points.push_back(transformed_point);
    }
    
    // 如果有法向量，也需要变换
    if (pointcloud->HasNormal()) {
        std::vector<Point3f>& normals = pointcloud->GetNorms();
        std::vector<Point3f>& transformed_normals = transformed_pointcloud->GetNorms();
        
        for (const auto& normal : normals) {
            Point3f transformed_normal;
            // 只应用旋转，不应用平移
            transformed_normal.x = normal.x * transform_matrix[0] + normal.y * transform_matrix[4] + normal.z * transform_matrix[8];
            transformed_normal.y = normal.x * transform_matrix[1] + normal.y * transform_matrix[5] + normal.z * transform_matrix[9];
            transformed_normal.z = normal.x * transform_matrix[2] + normal.y * transform_matrix[6] + normal.z * transform_matrix[10];
            transformed_normals.push_back(transformed_normal);
        }
    }

    return transformed_pointcloud;
}

/**
 * @brief 设置扫描参数
 * @param exposure_time 曝光时间
 * @param trigger_delay 触发延迟
 * @param repeat 重复次数
 * @param trigger_num 触发数量
 * @return 设置是否成功
 */
bool Scanner::set_scan_parameters(float exposure_time, float trigger_delay, int repeat, int trigger_num) {
    // 检查扫描仪是否已初始化
    if (!is_initialized_) {
        std::cout << "[错误] 扫描仪未初始化" << std::endl;
        return false;
    }

    return TJST3DSetScanerPar(exposure_time, trigger_delay, repeat, trigger_num, 0, 0);
}

/**
 * @brief 设置LED亮度
 * @param light LED亮度值(0-255)
 * @return 设置是否成功
 */
bool Scanner::set_led_light(unsigned char light) {
    // 检查扫描仪是否已初始化
    if (!is_initialized_) {
        std::cout << "[错误] 扫描仪未初始化" << std::endl;
        return false;
    }

    return TJST3DSetLedLight(light);
}

/**
 * @brief 保存点云到文件
 * @param filename 文件名
 * @return 保存是否成功
 */
bool Scanner::save_pointcloud(const std::string& filename) {
    // 检查扫描仪是否已初始化
    if (!is_initialized_) {
        std::cout << "[错误] 扫描仪未初始化" << std::endl;
        return false;
    }

    return TJST3DSavePointcloudTofileA(nullptr, filename.c_str());
}

/**
 * @brief 从文件加载点云
 * @param filename 文件名
 * @return 加载是否成功
 */
bool Scanner::load_pointcloud(const std::string& filename) {
    // 检查扫描仪是否已初始化
    if (!is_initialized_) {
        std::cout << "[错误] 扫描仪未初始化" << std::endl;
        return false;
    }

    return TJST3DLoadPointcloudFromfileA(nullptr, filename.c_str());
}

/**
 * @brief 将SDK的点云数据转换为Open3D点云
 * @param sdk_pointcloud SDK点云数据
 * @return Open3D点云对象
 */
std::shared_ptr<open3d::geometry::PointCloud> Scanner::convert_to_open3d_pointcloud(
    sn3DCore::sn3DRangeData* sdk_pointcloud) {
    // 检查输入点云是否有效
    if (!sdk_pointcloud) {
        return nullptr;
    }

    // 创建Open3D点云对象
    auto pointcloud = std::make_shared<open3d::geometry::PointCloud>();
    
    // 获取点云数量
    unsigned int num_points = TJST3DGetRangePoints(sdk_pointcloud);
    
    // 分配内存
    std::vector<float> points_x(num_points);
    std::vector<float> points_y(num_points);
    std::vector<float> points_z(num_points);
    
    // 获取点云数据
    TJST3DConvertRangePointToFloatArray(sdk_pointcloud, 
        points_x.data(), points_y.data(), points_z.data(), num_points);
    
    // 转换为Open3D点云格式
    pointcloud->points_.resize(num_points);
    for (size_t i = 0; i < num_points; ++i) {
        pointcloud->points_[i] = Eigen::Vector3d(points_x[i], points_y[i], points_z[i]);
    }
    
    return pointcloud;
}

/**
 * @brief 将Open3D点云转换为SDK点云数据
 * @param open3d_pointcloud Open3D点云对象
 * @return SDK点云数据
 */
sn3DCore::sn3DRangeData* Scanner::convert_to_sdk_pointcloud(
    const std::shared_ptr<open3d::geometry::PointCloud>& open3d_pointcloud) {
    // 检查输入点云是否有效
    if (!open3d_pointcloud || open3d_pointcloud->IsEmpty()) {
        return nullptr;
    }

    // 创建SDK点云对象
    auto sdk_pointcloud = new sn3DCore::sn3DRangeData();
    
    // 获取点云数量
    size_t num_points = open3d_pointcloud->points_.size();
    
    // 分配内存
    std::vector<float> points_x(num_points);
    std::vector<float> points_y(num_points);
    std::vector<float> points_z(num_points);
    
    // 转换点云数据
    for (size_t i = 0; i < num_points; ++i) {
        const auto& point = open3d_pointcloud->points_[i];
        points_x[i] = static_cast<float>(point.x());
        points_y[i] = static_cast<float>(point.y());
        points_z[i] = static_cast<float>(point.z());
    }
    
    // 设置SDK点云数据
    TJST3DSetRangePoints(sdk_pointcloud, num_points);
    TJST3DSetRangePointArray(sdk_pointcloud, 
        points_x.data(), points_y.data(), points_z.data(), num_points);
    
    return sdk_pointcloud;
}

/**
 * @brief 使用SDK方法融合多个点云数据
 * @param pointclouds 要融合的点云数据列表
 * @param merge_distance 融合距离阈值
 * @param filter_distance 滤波距离阈值
 * @return 融合后的点云数据
 */
std::shared_ptr<open3d::geometry::PointCloud> Scanner::merge_pointclouds_sdk(
    const std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& pointclouds,
    float merge_distance,
    float filter_distance) {
    
    std::cout << "[融合] 使用SDK方法开始融合多个点云..." << std::endl;
    
    if (pointclouds.empty()) {
        std::cout << "[错误] 没有输入点云" << std::endl;
        return nullptr;
    }

    // 创建SDK点云对象数组
    std::vector<sn3DCore::sn3DRangeData*> sdk_pointclouds;
    for (const auto& cloud : pointclouds) {
        if (!cloud || cloud->IsEmpty()) {
            continue;
        }
        auto sdk_cloud = convert_to_sdk_pointcloud(cloud);
        if (sdk_cloud) {
            sdk_pointclouds.push_back(sdk_cloud);
        }
    }

    if (sdk_pointclouds.empty()) {
        std::cout << "[错误] 没有有效的点云数据" << std::endl;
        return nullptr;
    }

    // 创建结果点云对象
    sn3DCore::sn3DRangeData* merged_sdk_cloud = new sn3DCore::sn3DRangeData();
    
    // 设置融合参数
    TJST3DSetMergeDistance(merge_distance);
    TJST3DSetFilterDistance(filter_distance);
    
    // 执行点云融合
    bool success = TJST3DMergePointclouds(
        sdk_pointclouds.data(),
        sdk_pointclouds.size(),
        merged_sdk_cloud
    );

    // 清理SDK点云资源
    for (auto cloud : sdk_pointclouds) {
        TJST3DRangeDelete(cloud);
    }

    if (!success) {
        std::cout << "[错误] 点云融合失败" << std::endl;
        TJST3DRangeDelete(merged_sdk_cloud);
        return nullptr;
    }

    // 转换为Open3D点云
    auto result_cloud = convert_to_open3d_pointcloud(merged_sdk_cloud);
    TJST3DRangeDelete(merged_sdk_cloud);

    std::cout << "[融合] 点云融合完成，最终点云包含 " << result_cloud->points_.size() << " 个点" << std::endl;
    return result_cloud;
}

/**
 * @brief 使用SDK方法对点云进行滤波处理
 * @param pointcloud 输入点云
 * @param filter_distance 滤波距离阈值
 * @return 滤波后的点云数据
 */
std::shared_ptr<open3d::geometry::PointCloud> Scanner::filter_pointcloud_sdk(
    const std::shared_ptr<open3d::geometry::PointCloud>& pointcloud,
    float filter_distance) {
    
    if (!pointcloud || pointcloud->IsEmpty()) {
        return nullptr;
    }

    // 转换为SDK点云
    auto sdk_cloud = convert_to_sdk_pointcloud(pointcloud);
    if (!sdk_cloud) {
        return nullptr;
    }

    // 创建结果点云对象
    sn3DCore::sn3DRangeData* filtered_sdk_cloud = new sn3DCore::sn3DRangeData();
    
    // 设置滤波参数
    TJST3DSetFilterDistance(filter_distance);
    
    // 执行点云滤波
    bool success = TJST3DFilterPointcloud(sdk_cloud, filtered_sdk_cloud);

    // 清理SDK点云资源
    TJST3DRangeDelete(sdk_cloud);

    if (!success) {
        std::cout << "[错误] 点云滤波失败" << std::endl;
        TJST3DRangeDelete(filtered_sdk_cloud);
        return nullptr;
    }

    // 转换为Open3D点云
    auto result_cloud = convert_to_open3d_pointcloud(filtered_sdk_cloud);
    TJST3DRangeDelete(filtered_sdk_cloud);

    std::cout << "[滤波] 点云滤波完成，结果点云包含 " << result_cloud->points_.size() << " 个点" << std::endl;
    return result_cloud;
}

/**
 * @brief 融合多个点云数据
 * @param pointclouds 要融合的点云数据列表
 * @param voxel_size 体素大小，用于降采样
 * @param distance_threshold 距离阈值，用于去除离群点
 * @param nb_neighbors 邻居点数量，用于统计滤波
 * @param std_ratio 标准差比率，用于统计滤波
 * @return 融合后的点云数据
 */
std::shared_ptr<open3d::geometry::PointCloud> Scanner::merge_pointclouds(
    const std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& pointclouds,
    double voxel_size,
    double distance_threshold,
    int nb_neighbors,
    double std_ratio) {
    
    std::cout << "[融合] 开始融合多个点云..." << std::endl;
    
    if (pointclouds.empty()) {
        std::cout << "[错误] 没有输入点云" << std::endl;
        return nullptr;
    }

    // 创建结果点云
    auto merged_cloud = std::make_shared<open3d::geometry::PointCloud>();
    
    // 逐个处理每个点云
    for (const auto& cloud : pointclouds) {
        if (!cloud || cloud->IsEmpty()) {
            continue;
        }

        // 对每个点云进行滤波
        auto filtered_cloud = filter_pointcloud(cloud, voxel_size, distance_threshold, 
            nb_neighbors, std_ratio);
        
        if (!filtered_cloud || filtered_cloud->IsEmpty()) {
            continue;
        }

        // 合并点云
        *merged_cloud += *filtered_cloud;
    }

    // 对合并后的点云进行最终滤波
    if (!merged_cloud->IsEmpty()) {
        merged_cloud = filter_pointcloud(merged_cloud, voxel_size, distance_threshold,
            nb_neighbors, std_ratio);
    }

    std::cout << "[融合] 点云融合完成，最终点云包含 " << merged_cloud->points_.size() << " 个点" << std::endl;
    return merged_cloud;
}

/**
 * @brief 对点云进行滤波处理
 * @param pointcloud 输入点云
 * @param voxel_size 体素大小，用于降采样
 * @param distance_threshold 距离阈值，用于去除离群点
 * @param nb_neighbors 邻居点数量，用于统计滤波
 * @param std_ratio 标准差比率，用于统计滤波
 * @return 滤波后的点云数据
 */
std::shared_ptr<open3d::geometry::PointCloud> Scanner::filter_pointcloud(
    const std::shared_ptr<open3d::geometry::PointCloud>& pointcloud,
    double voxel_size,
    double distance_threshold,
    int nb_neighbors,
    double std_ratio) {
    
    if (!pointcloud || pointcloud->IsEmpty()) {
        return nullptr;
    }

    // 1. 体素降采样
    auto downsampled_cloud = downsample_pointcloud(pointcloud, voxel_size);
    
    // 2. 统计滤波去除离群点
    auto filtered_cloud = remove_outliers(downsampled_cloud, nb_neighbors, std_ratio);
    
    return filtered_cloud;
}

/**
 * @brief 使用统计滤波去除离群点
 * @param pointcloud 输入点云
 * @param nb_neighbors 邻居点数量
 * @param std_ratio 标准差比率
 * @return 滤波后的点云
 */
std::shared_ptr<open3d::geometry::PointCloud> Scanner::remove_outliers(
    const std::shared_ptr<open3d::geometry::PointCloud>& pointcloud,
    int nb_neighbors,
    double std_ratio) {
    
    if (!pointcloud || pointcloud->IsEmpty()) {
        return nullptr;
    }

    // 创建KD树用于快速近邻搜索
    auto kdtree = std::make_shared<open3d::geometry::KDTreeFlann>(*pointcloud);
    
    // 存储每个点到其邻居的平均距离
    std::vector<double> distances(pointcloud->points_.size());
    
    // 计算每个点到其邻居的平均距离
    #pragma omp parallel for
    for (int i = 0; i < pointcloud->points_.size(); ++i) {
        std::vector<int> indices;
        std::vector<double> dists;
        kdtree->SearchKNN(pointcloud->points_[i], nb_neighbors, indices, dists);
        
        // 计算平均距离
        double avg_dist = 0.0;
        for (double dist : dists) {
            avg_dist += std::sqrt(dist);
        }
        avg_dist /= dists.size();
        distances[i] = avg_dist;
    }
    
    // 计算距离的均值和标准差
    double mean = 0.0;
    for (double dist : distances) {
        mean += dist;
    }
    mean /= distances.size();
    
    double std_dev = 0.0;
    for (double dist : distances) {
        std_dev += (dist - mean) * (dist - mean);
    }
    std_dev = std::sqrt(std_dev / distances.size());
    
    // 创建新的点云，只保留在阈值范围内的点
    auto filtered_cloud = std::make_shared<open3d::geometry::PointCloud>();
    double threshold = mean + std_ratio * std_dev;
    
    for (size_t i = 0; i < pointcloud->points_.size(); ++i) {
        if (distances[i] <= threshold) {
            filtered_cloud->points_.push_back(pointcloud->points_[i]);
            if (!pointcloud->colors_.empty()) {
                filtered_cloud->colors_.push_back(pointcloud->colors_[i]);
            }
            if (!pointcloud->normals_.empty()) {
                filtered_cloud->normals_.push_back(pointcloud->normals_[i]);
            }
        }
    }
    
    return filtered_cloud;
}

/**
 * @brief 使用体素网格进行降采样
 * @param pointcloud 输入点云
 * @param voxel_size 体素大小
 * @return 降采样后的点云
 */
std::shared_ptr<open3d::geometry::PointCloud> Scanner::downsample_pointcloud(
    const std::shared_ptr<open3d::geometry::PointCloud>& pointcloud,
    double voxel_size) {
    
    if (!pointcloud || pointcloud->IsEmpty()) {
        return nullptr;
    }

    // 创建体素网格
    auto voxel_grid = open3d::geometry::VoxelGrid::CreateFromPointCloud(*pointcloud, voxel_size);
    
    // 从体素网格中提取点云
    auto downsampled_cloud = std::make_shared<open3d::geometry::PointCloud>();
    for (const auto& voxel : voxel_grid->voxels_) {
        downsampled_cloud->points_.push_back(voxel.second);
    }
    
    return downsampled_cloud;
}

/**
 * @brief 进入扫描模式
 * @return 是否成功进入扫描模式
 */
bool Scanner::enter_scan_mode() {
    if (!is_initialized_) {
        std::cout << "[错误] 扫描仪未初始化" << std::endl;
        return false;
    }

    if (!TJST3DEnterScanMode()) {
        std::cout << "[错误] 进入扫描模式失败" << std::endl;
        return false;
    }

    std::cout << "[信息] 成功进入扫描模式" << std::endl;
    return true;
}

/**
 * @brief 显示点云数据
 * @param pointcloud 要显示的点云数据
 * @return 是否成功显示点云
 */
bool Scanner::show_pointcloud(sn3DCore::sn3DRangeData* pointcloud) {
    if (!pointcloud) {
        std::cout << "[错误] 点云数据为空" << std::endl;
        return false;
    }

    // 显示点云
    bool success = TJST3DShowPoint(pointcloud);

    if (!success) {
        std::cout << "[错误] 显示点云失败" << std::endl;
        return false;
    }

    std::cout << "[信息] 点云显示成功" << std::endl;
    return true;
}

/**
 * @brief 扫描单个点云（多角度扫描中的一个）
 * @return 是否成功扫描
 */
bool Scanner::scan_single_pointcloud() {
    if (!is_initialized_) {
        std::cout << "[错误] 扫描仪未初始化" << std::endl;
        return false;
    }

    // 创建新的点云数据
    sn3DCore::sn3DRangeData* new_cloud = new sn3DCore::sn3DRangeData();
    
    // 执行扫描
    if (!TJST3DCreatePointcloudFromScan(camera_group_id_, new_cloud)) {
        std::cout << "[错误] 扫描点云失败" << std::endl;
        TJST3DRangeDelete(new_cloud);
        return false;
    }

    // 保存点云数据
    scanned_pointclouds_.push_back(new_cloud);
    std::cout << "[信息] 成功扫描第 " << scanned_pointclouds_.size() << " 个点云" << std::endl;
    return true;
}

/**
 * @brief 融合所有已扫描的点云
 * @param output_file 输出文件名
 * @return 是否成功融合
 */
bool Scanner::merge_scanned_pointclouds(const std::string& output_file) {
    if (scanned_pointclouds_.empty()) {
        std::cout << "[错误] 没有可融合的点云数据" << std::endl;
        return false;
    }

    // 创建输出点云
    sn3DCore::sn3DRangeData* merged_cloud = new sn3DCore::sn3DRangeData();

    // 执行点云融合
    if (!TJST3DMergePointcloud(scanned_pointclouds_.size(), scanned_pointclouds_.data(), merged_cloud)) {
        std::cout << "[错误] 点云融合失败" << std::endl;
        TJST3DRangeDelete(merged_cloud);
        return false;
    }

    // 保存融合后的点云
    if (!TJST3DSavePointcloudTofileA(merged_cloud, output_file.c_str())) {
        std::cout << "[错误] 保存融合点云失败" << std::endl;
        TJST3DRangeDelete(merged_cloud);
        return false;
    }

    // 清理
    TJST3DRangeDelete(merged_cloud);
    std::cout << "[信息] 成功融合 " << scanned_pointclouds_.size() << " 个点云并保存到 " << output_file << std::endl;
    return true;
}

/**
 * @brief 清除所有已扫描的点云
 */
void Scanner::clear_scanned_pointclouds() {
    // 清理所有点云数据
    for (auto* cloud : scanned_pointclouds_) {
        if (cloud) {
            TJST3DRangeDelete(cloud);
        }
    }
    scanned_pointclouds_.clear();
    std::cout << "[信息] 已清除所有扫描的点云数据" << std::endl;
} 