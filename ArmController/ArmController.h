 #ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <vector>
#include <memory>

// 前向声明
class SDK;

class ArmController {
public:
    explicit ArmController(std::shared_ptr<SDK> sdk);
    
    // 移动机械臂到初始位置
    bool moveToHome();
    
    // 控制夹爪
    bool openGripper();
    bool closeGripper();
    
    // 执行轨迹
    bool executeTrajectory(const std::vector<std::vector<double>>& pathPoints);
    
    // 规划安全路径
    std::vector<std::vector<double>> planSafePath(const std::vector<double>& targetPose);

private:
    std::shared_ptr<SDK> sdk_;  // SDK接口对象
};

#endif // ARM_CONTROLLER_H