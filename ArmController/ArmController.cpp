 #include "ArmController.h"
#include <iostream>

ArmController::ArmController(std::shared_ptr<SDK> sdk) : sdk_(sdk) {}

bool ArmController::moveToHome() {
    // 示例值
    std::vector<double> homePose = {0, -90, 90, 0, 90, 0};
    return sdk_->moveJoints(homePose);
}

bool ArmController::openGripper() {
    return sdk_->controlGripper(true);
}

bool ArmController::closeGripper() {
    return sdk_->controlGripper(false);
}

bool ArmController::executeTrajectory(const std::vector<std::vector<double>>& pathPoints) {
    for (const auto& pt : pathPoints) {
        if (!sdk_->moveCartesian(pt)) {
            std::cout << "[执行中断] 点位不可达或受限" << std::endl;
            return false;
        }
    }
    return true;
}

std::vector<std::vector<double>> ArmController::planSafePath(const std::vector<double>& targetPose) {
    // 伪代码：实际应接入 RRT*/A* 等避障算法
    return {targetPose};
}