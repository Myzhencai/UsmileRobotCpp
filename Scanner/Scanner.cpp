#include "Scanner.h"
#include <iostream>
#include <array>

Scanner::Scanner(bool is_initialized, int camera_group_id)
    : is_initialized_(is_initialized), camera_group_id_(camera_group_id) {}

/**
 * @brief 初始化扫描仪
 * @param mode 设备模式：0-USB设备，1-网络设备
 * @param dev_num 设备数量
 * @return 初始化是否成功
 */
bool Scanner::initialize(int mode, int dev_num)
{
    // 检查是否已经初始化
    // if (usb_mode_) {
    //     // std::cout << "扫描仪已经初始化" << std::endl;
    //     return true;
    // }

    // 初始化SDK
    if (!TJST3DModuleInit(mode, dev_num))
    {
        // std::cout << "扫描仪初始化失败" << std::endl;
        return false;
    }

    // 设置默认参数
    TJST3DSetProcessorUnit(1); // 使用GPU进行点云处理
    TJST3DSetModuleType(0);    // 设置为双目模式
    TJST3DSetWorkMode(0);      // 设置为扫描模式

    // usb_mode_ = true;
    // std::cout << "扫描仪初始化成功" << std::endl;
    return true;
}

/**
 * @brief 释放扫描仪资源
 */
void Scanner::uninitialize()
{
    if (!is_initialized_)
    {
        return;
    }

    // 释放SDK资源
    TJST3DModuleUninit();
    is_initialized_ = false;
    std::cout << "扫描仪资源已释放" << std::endl;
}

/**
 * @brief 设置扫描参数
 * @param exposure_time 曝光时间
 * @param trigger_delay 触发延迟
 * @param repeat 重复次数
 * @param trigger_num 触发数量
 * @return 设置是否成功
 */
bool Scanner::set_scan_parameters(float exposure_time, float trigger_delay, int repeat, int trigger_num)
{
    // 检查扫描仪是否已初始化
    if (!is_initialized_)
    {
        // std::cout<<"扫描仪未初始化" << std::endl;
        return false;
    }

    return TJST3DSetScanerPar(exposure_time, trigger_delay, repeat, trigger_num, 0, 0);
}

/**
 * @brief 设置LED亮度
 * @param light LED亮度值(0-255)
 * @return 设置是否成功
 */
bool Scanner::set_led_light(unsigned char light)
{
    // 检查扫描仪是否已初始化
    if (!is_initialized_)
    {
        // std::cout << "扫描仪未初始化" << std::endl;
        return false;
    }

    return TJST3DSetLedLight(light);
}

/**
 * @brief 保存点云到文件
 * @param filename 文件名
 * @return 保存是否成功
 */
bool Scanner::save_pointcloud(const std::string &filename)
{
    // 检查扫描仪是否已初始化
    if (!is_initialized_)
    {
        // std::cout << "扫描仪未初始化" << std::endl;
        return false;
    }

    sn3DCore::sn3DRangeData *pointcloud = new sn3DCore::sn3DRangeData();
    bool result = TJST3DSavePointcloudTofileA(pointcloud, filename.c_str());
    delete pointcloud;
    return result;
}

/**
 * @brief 从文件加载点云
 * @param filename 文件名
 * @return 加载是否成功
 */
bool Scanner::load_pointcloud(const std::string &filename)
{
    // 检查扫描仪是否已初始化
    if (!is_initialized_)
    {
        // std::cout << "扫描仪未初始化" << std::endl;
        return false;
    }

    sn3DCore::sn3DRangeData *pointcloud = new sn3DCore::sn3DRangeData();
    bool result = TJST3DLoadPointcloudFromfileA(pointcloud, filename.c_str());
    delete pointcloud;
    return result;
}

/**
 * @brief 进入扫描模式
 * @return 是否成功进入扫描模式
 */
bool Scanner::enter_scan_mode()
{
    // 检查扫描仪是否已初始化
    if (!is_initialized_)
    {
        // std::cout << "扫描仪未初始化" << std::endl;
        return false;
    }

    // 进入扫描模式
    return TJST3DEnterScanMode();
}

/**
 * @brief 显示点云数据
 * @param pointcloud 要显示的点云数据
 * @return 是否成功显示点云
 */
bool Scanner::show_pointcloud(sn3DCore::sn3DRangeData *pointcloud)
{
    // 检查扫描仪是否已初始化
    if (!is_initialized_)
    {
        // std::cout << "ok" << std::endl;
        return false;
    }

    if (!pointcloud)
    {
        // std::cout << "要显示的点云为空" << std::endl;
        return false;
    }

    return TJST3DShowPoint(pointcloud);
}

/**
 * @brief 扫描单个点云（多角度扫描中的一个）
 * @return 是否成功扫描
 */
bool Scanner::scan_single_pointcloud()
{
    // 检查扫描仪是否已初始化
    std::cout << "scan_single_pointcloud" << std::endl;
    if (!is_initialized_)
    {
        // std::cout << "扫描仪未初始化" << std::endl;
        return false;
    }

    // 创建新的点云数据
    sn3DCore::sn3DRangeData *new_cloud = new sn3DCore::sn3DRangeData();

    // 执行扫描
    if (!TJST3DCreatePointcloudFromScan(camera_group_id_, new_cloud))
    {
        // std::cout << "扫描点云失败" << std::endl;
        TJST3DRangeDelete(new_cloud);
        return false;
    }

    // 保存点云数据
    scanned_pointclouds_.push_back(new_cloud);
    // std::cout << "成功扫描第" << scanned_pointclouds_.size() << "个点云" << std::endl;
    return true;
}

/**
 * @brief 融合所有已扫描的点云
 * @param output_file 输出文件名
 * @return 是否成功融合
 */
bool Scanner::merge_scanned_pointclouds(const std::string &output_file)
{
    if (scanned_pointclouds_.empty())
    {
        std::cout << "scanned_pointclouds_ empty" << std::endl;
        return false;
    }

    // 1. 创建输出点云
    sn3DCore::sn3DRangeData *merged_cloud = new sn3DCore::sn3DRangeData();

    // 2. 进行点云配准
    float rot[16]; // 4x4转换矩阵
    float diff;    // 全局平均误差
    if (!TJST3DMatchPointcloudA(scanned_pointclouds_.size(), scanned_pointclouds_.data(), 0.1f, 1000, rot, diff, "config.ini"))
    {
        // std::cout << "点云配准失败" << std::endl;
        TJST3DRangeDelete(merged_cloud);
        return false;
    }

    // 3. 执行点云融合
    if (!TJST3DMergePointcloud(scanned_pointclouds_.size(), scanned_pointclouds_.data(), merged_cloud))
    {
        // std::cout << "点云融合失败" << std::endl;
        TJST3DRangeDelete(merged_cloud);
        return false;
    }

    // 4. 保存融合后的点云
    if (!TJST3DSavePointcloudTofileA(merged_cloud, output_file.c_str()))
    {
        // std::cout << "保存融合点云失败" << std::endl;
        TJST3DRangeDelete(merged_cloud);
        return false;
    }

    // 5. 清理内存
    TJST3DRangeDelete(merged_cloud);
    // std::cout << "成功融合 " << scanned_pointclouds_.size() << "个点云并保存到 " << output_file << std::endl;
    return true;
}

/**
 * @brief 清除所有已扫描的点云
 */
void Scanner::clear_scanned_pointclouds()
{
    for (auto &pointcloud : scanned_pointclouds_)
    {
        TJST3DRangeDelete(pointcloud);
    }
    scanned_pointclouds_.clear();
    std::cout << "clear all scanned_pointclouds_" << std::endl;
}

/**
 * @brief 抓取图片
 * @param left_image_filename 左相机图片保存文件名（可选）
 * @param right_image_filename 右相机图片保存文件名（可选）
 * @param bRGB 是否抓取彩色图，默认为true
 * @return 抓取是否成功
 */
bool Scanner::capture_image(const std::string &left_image_filename, const std::string &right_image_filename, bool bRGB)
{
    std::cout << "capture_image" << std::endl;
    if (!is_initialized_)
    {
        // std::cout << "扫描仪未初始化" << std::endl;
        return false;
    }

    int width = 1920, height = 1080, bitCount = 24;
    if (!TJST3DGetImageFormat(&width, &height, &bitCount))
    {
        // std::cout << "无法获取图像格式" << std::endl;
        return false;
    }

    // 计算图像缓冲区大小
    // unsigned int image_buffer_size = width * height * (bitCount / 8);
    const unsigned int image_buffer_size = width * height * (bitCount / 8);
    unsigned char *left_image_buffer = new (std::nothrow) unsigned char[image_buffer_size];
    unsigned char *right_image_buffer = new (std::nothrow) unsigned char[image_buffer_size];

    if (!left_image_buffer || !right_image_buffer)
    {
        // std::cout << "内存分配失败" << std::endl;
        delete[] left_image_buffer;
        delete[] right_image_buffer;
        return false;
    }

    unsigned int caught_size = TJST3DCatchImageEx(left_image_buffer, right_image_buffer, image_buffer_size, bRGB);
    if (caught_size == 0)
    {
        // std::cout << "抓取图片失败" << std::endl;
        delete[] left_image_buffer;
        delete[] right_image_buffer;
        return false;
    }

    bool success = true;
    if (!left_image_filename.empty())
    {
        if (!TJST3DSaveBmpImageA(left_image_filename.c_str(), left_image_buffer, bitCount))
        {
            // std::cout << "保存左相机图片失败: " << left_image_filename << std::endl;
            success = false;
        }
        else
        {
            // std::cout << "左相机图片已保存到: " << left_image_filename << std::endl;
        }
    }

    if (!right_image_filename.empty())
    {
        if (!TJST3DSaveBmpImageA(right_image_filename.c_str(), right_image_buffer, bitCount))
        {
            // std::cout << "保存右相机图片失败: " << right_image_filename << std::endl;
            success = false;
        }
        else
        {
            // std::cout << "右相机图片已保存到: " << right_image_filename << std::endl;
        }
    }

    delete[] left_image_buffer;
    delete[] right_image_buffer;
    return success;
}

/**
 * @brief 根据给定的平移和旋转对点云进行变换
 * @param pointcloud 输入点云数据
 * @param translation 平移向量 (x, y, z)
 * @param rotation 旋转欧拉角 (rx, ry, rz)
 * @return 变换后的点云数据
 */
sn3DCore::sn3DRangeData *Scanner::transform_pointcloud(
    sn3DCore::sn3DRangeData *pointcloud,
    const std::array<double, 3> &translation,
    const std::array<double, 3> &rotation)
{
    std::cout << "transform_pointcloud" << std::endl;

    // 检查输入点云是否有效
    if (!pointcloud)
    {
        std::cout << "[错误] 输入点云为空" << std::endl;
        return nullptr;
    }

    // 创建新的点云对象用于存储变换结果
    sn3DCore::sn3DRangeData *transformed_pointcloud = new sn3DCore::sn3DRangeData();

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
    std::vector<Point3f> &points = pointcloud->GetPoints();
    std::vector<Point3f> &transformed_points = transformed_pointcloud->GetPoints();

    // 复制点云基本属性
    transformed_pointcloud->SetMap(pointcloud->GetW(), pointcloud->GetH());

    // 对每个点应用变换矩阵
    for (const auto &point : points)
    {
        Point3f transformed_point;
        // 应用旋转和平移
        transformed_point.x = point.x * transform_matrix[0] + point.y * transform_matrix[4] + point.z * transform_matrix[8] + transform_matrix[12];
        transformed_point.y = point.x * transform_matrix[1] + point.y * transform_matrix[5] + point.z * transform_matrix[9] + transform_matrix[13];
        transformed_point.z = point.x * transform_matrix[2] + point.y * transform_matrix[6] + point.z * transform_matrix[10] + transform_matrix[14];
        transformed_points.push_back(transformed_point);
    }

    // 如果有法向量，也需要变换
    if (pointcloud->HasNormal())
    {
        std::vector<Point3f> &normals = pointcloud->GetNorms();
        std::vector<Point3f> &transformed_normals = transformed_pointcloud->GetNorms();

        for (const auto &normal : normals)
        {
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
 * @brief 析构函数
 */
Scanner::~Scanner()
{
    clear_scanned_pointclouds();
    if (is_initialized_)
    {
        uninitialize();
    }
}

// std::string Scanner::getDeviceInfo() const {
//     return "Scanner Device ID: " + std::to_string(camera_group_id_);
// }

int main()
{
    Scanner scanner(true, 1);
    scanner.initialize(0, 1);
    scanner.enter_scan_mode();
    // scanner.set_scan_parameters(100., 15., 10, 20);
    // scanner.set_led_light(128);
    // scanner.save_pointcloud("output.ply");
    // // scanner.load_pointcloud("input.ply");
    // // 创建一个点云对象并显示
    // sn3DCore::sn3DRangeData pointcloud;
    // scanner.show_pointcloud(&pointcloud);

    scanner.scan_single_pointcloud();
    scanner.merge_scanned_pointclouds("merged_output.ply");
    scanner.clear_scanned_pointclouds();

    scanner.capture_image("left_image.bmp", "right_image.bmp", true);

    std::array<double, 3> translation = {0.0, 0.0, 0.0};
    std::array<double, 3> rotation = {0.0, 0.0, 0.0};
    scanner.transform_pointcloud(nullptr, translation, rotation);

    // 初始化SDK
    // if (!TJST3DModuleInit(0, 1)) {
    //     // std::cout << "扫描仪初始化失败" << std::endl;
    //     return false;
    // }

    // // 设置默认参数
    // TJST3DSetProcessorUnit(1);  // 使用GPU进行点云处理
    // TJST3DSetModuleType(0);     // 设置为双目模式
    // TJST3DSetWorkMode(0);       // 设置为扫描模式
    scanner.uninitialize();
    // scanner.~Scanner();
    // std::cout << scanner.getDeviceInfo() << std::endl;
    return 0;
}
