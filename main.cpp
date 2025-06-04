#include <iostream>
#include <string>
#include <memory>
#include "SystemStateMachine/StateMachine.h"
#include "DeviceManager/DeviceManager.h"
#include "ArmController/ArmController.h"
#include "CalibrationManager/CalibrationManager.h"
#include "Scanner/Scanner.h"
#include "TrajectoryPlanner/TrajectoryPlanner.h"
#include "ForceMonitor/ForceMonitor.h"
#include "CleaningEvaluator/CleaningEvaluator.h"

int main() {
    // 创建各个模块的实例
    auto stateMachine = std::make_unique<StateMachine>();
    auto deviceManager = std::make_unique<DeviceManager>();
    auto armController = std::make_unique<ArmController>();
    auto calibrationManager = std::make_unique<CalibrationManager>();
    auto scanner = std::make_unique<Scanner>();
    auto trajectoryPlanner = std::make_unique<TrajectoryPlanner>();
    auto forceMonitor = std::make_unique<ForceMonitor>();
    auto cleaningEvaluator = std::make_unique<CleaningEvaluator>();

    std::cout << "系统启动中..." << std::endl;

    // 步骤1：初始化与连接设备
    if (!(deviceManager->connectArm() && deviceManager->connectCamera() && deviceManager->connectForceSensor())) {
        std::cout << "设备连接失败，退出程序" << std::endl;
        return 1;
    }
    if (!deviceManager->checkDeviceStatus()) {
        deviceManager->handleCommunicationError();
        return 1;
    }

    std::cout << "设备连接成功，状态正常" << std::endl;

    // 步骤2：机械臂回归安全位置
    if (!armController->returnToSafePosition()) {
        std::cout << "机械臂无法回归初始位置，退出" << std::endl;
        return 1;
    }

    // 步骤3：判断是否需要更换末端装置（人工装配）
    std::string userInput;
    std::cout << "是否更换牙刷/冲牙器？(y/n): ";
    std::cin >> userInput;
    if (userInput == "y" || userInput == "Y") {
        armController->releaseGripper();
        std::cout << "请人工更换装置，按回车继续...";
        std::cin.ignore();
        std::cin.get();
        armController->gripGripper();
        calibrationManager->calibrateEndEffector();
    }

    // 步骤4：牙模扫描
    scanner->scanDentalModel();
    scanner->buildPointCloud();
    scanner->calculateDentalModelCoordinates();

    // 步骤5：用户指定刷洗区域与参数
    std::string area;
    std::cout << "请输入需要刷洗的区域编号（如left_upper）：";
    std::cin >> area;
    
    struct CleaningParameters {
        std::string frequency = "中";
        int duration = 10;
        std::string force = "中";
    } params;

    auto cleaningTrajectory = trajectoryPlanner->generateCleaningTrajectory(area, params);

    // 步骤6：执行刷洗任务 + 实时监控力反馈
    const double forceThreshold = 10.0; // 示例力阈值，需要根据实际情况调整
    for (const auto& point : cleaningTrajectory) {
        armController->executeTrajectory(point);
        double force = forceMonitor->monitorForce();
        if (forceMonitor->detectOverload() || force > forceThreshold) {
            std::cout << "检测到过载/异常力，暂停机械臂" << std::endl;
            break;
        }
        forceMonitor->dynamicAdjustment();
    }

    // 步骤7：清洁效果评估
    std::string beforeImage = "img/before.jpg";
    std::string afterImage = "img/after.jpg";
    double cleaningIndex = cleaningEvaluator->calculateCleaningIndex(beforeImage, afterImage);
    std::cout << "清洁指数为：" << cleaningIndex << std::endl;

    std::cout << "自动刷牙流程完成。" << std::endl;
    return 0;
} 