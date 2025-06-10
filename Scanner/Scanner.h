#pragma once

#include <memory>
#include <vector>
#include <string>
#include <array>
#include "TJST3DScannerApi.h"

/**
 * @brief 扫描仪类，用于控制3D扫描仪硬件和处理点云数据
 */
class Scanner {
public:
    /**
     * @brief 构造函数
     */
    Scanner();

    /**
     * @brief 析构函数
     */
    ~Scanner();

    /**
     * @brief 初始化扫描仪
     * @param mode 设备模式：0-USB设备，1-网络设备
     * @param dev_num 设备数量
     * @return 初始化是否成功
     */
    bool initialize(int mode = 0, int dev_num = 1);

    /**
     * @brief 释放扫描仪资源
     */
    void uninitialize();

     /**
     * @brief 根据给定的平移和旋转对点云进行变换
     * @param pointcloud 输入点云数据
     * @param translation 平移向量 (x, y, z)
     * @param rotation 旋转欧拉角 (rx, ry, rz)
     * @return 变换后的点云数据
     */
    sn3DCore::sn3DRangeData* transform_pointcloud(
        sn3DCore::sn3DRangeData* pointcloud,
        const std::array<double, 3>& translation,
        const std::array<double, 3>& rotation);


    /**
     * @brief 设置扫描参数
     * @param exposure_time 曝光时间
     * @param trigger_delay 触发延迟
     * @param repeat 重复次数
     * @param trigger_num 触发数量
     * @return 设置是否成功
     */
    bool set_scan_parameters(float exposure_time, float trigger_delay, int repeat, int trigger_num);

    /**
     * @brief 设置LED亮度
     * @param light LED亮度值(0-255)
     * @return 设置是否成功
     */
    bool set_led_light(unsigned char light);

    /**
     * @brief 保存点云到文件
     * @param filename 文件名
     * @return 保存是否成功
     */
    bool save_pointcloud(const std::string& filename);

    /**
     * @brief 从文件加载点云
     * @param filename 文件名
     * @return 加载是否成功
     */
    bool load_pointcloud(const std::string& filename);

    /**
     * @brief 进入扫描模式
     * @return 是否成功进入扫描模式
     */
    bool enter_scan_mode();

    /**
     * @brief 显示点云数据
     * @param pointcloud 要显示的点云数据
     * @return 是否成功显示点云
     */
    bool show_pointcloud(sn3DCore::sn3DRangeData* pointcloud);

    /**
     * @brief 扫描单个点云（多角度扫描中的一个）
     * @return 是否成功扫描
     */
    bool scan_single_pointcloud();

    /**
     * @brief 融合所有已扫描的点云
     * @param output_file 输出文件名
     * @return 是否成功融合
     */
    bool merge_scanned_pointclouds(const std::string& output_file);

    /**
     * @brief 清除所有已扫描的点云
     */
    void clear_scanned_pointclouds();

   

private:
    bool is_initialized_;                      ///< 扫描仪是否已初始化
    int camera_group_id_;                      ///< 相机组ID
    std::vector<sn3DCore::sn3DRangeData*> scanned_pointclouds_;  ///< 已扫描的点云数据
}; 