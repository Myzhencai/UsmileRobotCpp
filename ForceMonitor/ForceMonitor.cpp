#include "ForceMonitor.h"
#include <vector>
#include <cmath>

// ForceMonitor类的构造函数，初始化传感器指针和力阈值
ForceMonitor::ForceMonitor(std::shared_ptr<kw::SensorControl> forceSensor) 
    : sensor(forceSensor), forceThreshold(5.0) {}

// 读取当前的力数据，返回一个包含6个分量的double类型向量
std::vector<double> ForceMonitor::readForce() {
    float force[6];
    std::vector<double> result; 
    
    
    if (sensor->GetCurrentForceData(force) == 28) {
        // 将float数组转换为vector<double>
        result.assign(force, force + 6);
    }
    
    return result;
}

// 判断当前力是否超过阈值，超载返回true，否则返回false
bool ForceMonitor::isOverloaded() {
    auto force = readForce(); 
    if (force.empty()) return false; 
    // 遍历每个力分量，判断是否有超过阈值的情况
    for (const auto& f : force) {
        if (std::abs(f) > forceThreshold) {
            return true; // 只要有一个分量超过阈值即认为超载，这里需要给6个分量都给限制
        }
    }
    return false; 
}

// 根据力数据微调轨迹（当前未实现，直接返回原轨迹）
std::vector<double> ForceMonitor::adjustTrajectory(const std::vector<double>& trajectory) {
    // TODO: 根据力数据微调路径
    return trajectory;
} 