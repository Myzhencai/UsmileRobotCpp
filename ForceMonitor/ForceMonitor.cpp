#include "ForceMonitor.h"
#include <vector>
#include <cmath>

/**
 * @brief 构造函数实现
 * @param forceSensor 力传感器控制器的智能指针
 * @note 初始化传感器指针和力阈值，默认阈值设置为5.0
 */
ForceMonitor::ForceMonitor(std::shared_ptr<kw::SensorControl> forceSensor) 
    : sensor(forceSensor), forceThreshold(5.0) {}

/**
 * @brief 读取当前力传感器数据
 * @return 返回包含6个分量的力数据向量，分别对应x,y,z方向的力和力矩
 * @note 如果读取失败，返回空向量
 */
std::vector<double> ForceMonitor::readForce() {
    float force[6];
    std::vector<double> result; 
    
    // 调用传感器接口获取力数据，返回28表示成功
    if (sensor->GetCurrentForceData(force) == 28) {
        // 将float数组转换为vector<double>类型
        result.assign(force, force + 6);
    }
    
    return result;
}

/**
 * @brief 检查是否超过力阈值
 * @return 如果任一方向的力超过阈值返回true，否则返回false
 * @note 分别检查6个分量（3个方向的力和力矩），任一超过阈值即认为超载
 */
bool ForceMonitor::isOverloaded() {
    auto force = readForce(); 
    if (force.empty()) return false; 
    
    // 遍历每个力分量，判断是否有超过阈值的情况
    for (const auto& f : force) {
        if (std::abs(f) > forceThreshold) {
            return true; // 只要有一个分量超过阈值即认为超载
        }
    }
    return false; 
}

/**
 * @brief 根据力数据调整轨迹
 * @param trajectory 原始轨迹数据
 * @return 调整后的轨迹数据
 * @note 当前版本直接返回原始轨迹，待实现根据力数据调整轨迹的功能
 */
std::vector<double> ForceMonitor::adjustTrajectory(const std::vector<double>& trajectory) {
    // TODO: 根据力数据微调路径，需要考虑力反馈进行轨迹补偿
    return trajectory;
} 