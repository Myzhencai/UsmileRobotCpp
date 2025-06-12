#pragma once

#include <vector>
#include <memory>
#include "kw-lib-all.h"

/**
 * @brief 力传感器监控类
 * 用于监控和控制力传感器的状态，确保系统在安全范围内运行
 */
class ForceMonitor {
public:
    /**
     * @brief 构造函数
     * @param forceSensor 力传感器控制器的智能指针
     */
    ForceMonitor(std::shared_ptr<kw::SensorControl> forceSensor);

    /**
     * @brief 读取当前力传感器数据
     * @return 返回力传感器测量的力值向量
     */
    std::vector<double> readForce();

    /**
     * @brief 检查是否超过力阈值
     * @return 如果力值超过阈值返回true，否则返回false
     */
    bool isOverloaded();

    /**
     * @brief 根据力传感器数据调整轨迹
     * @param trajectory 原始轨迹数据
     * @return 调整后的轨迹数据
     */
    std::vector<double> adjustTrajectory(const std::vector<double>& trajectory);

private:
    std::shared_ptr<kw::SensorControl> sensor;    ///< 力传感器控制器
    double forceThreshold;                        ///< 力阈值，用于判断是否过载
}; 