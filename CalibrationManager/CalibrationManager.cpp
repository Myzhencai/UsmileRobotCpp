#include "CalibrationManager.hpp"
#include <iostream>

CalibrationManager::CalibrationManager(RobotSDK* robot, Camera* camera)
    : robot_(robot), camera_(camera) {
}

bool CalibrationManager::performEndToolCalibration() {
    std::cout << "[标定] 末端工具标定流程启动" << std::endl;
    // 结构光或外部测量流程
    return true;
}

bool CalibrationManager::performHandEyeCalibration() {
    std::cout << "[标定] 手眼标定流程启动" << std::endl;
    // 标定板拍照、solvePnP等过程
    return true;
} 