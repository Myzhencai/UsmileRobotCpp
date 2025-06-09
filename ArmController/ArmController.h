#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <string>
#include <vector>
#include <memory>
#include "CodroidApi/CodroidApi.h"

class ArmController {
public:
    ArmController(const std::string& ip = "192.168.101.100", const std::string& port = "9000");
    ~ArmController();

    // 基础控制接口
    bool connect();
    bool disconnect();
    bool isConnected() const;

    // 夹爪控制接
    bool openGripper(int port = 1);  // 打开夹爪，默认使用端口1
    bool closeGripper(int port = 1); // 关闭夹爪，默认使用端口1
    bool isGripperOpen(int port = 1) const; // 检查夹爪状态

    bool executeTrajectory(const MovCartSegments& segments);

    // Home position control
    bool setHomePosition(const std::vector<double>& position);  // Set home position with joint angles
    bool goHome(double speed = 60, double acc = 80);  // Move to home position
    MovCartSegments createHomeTrajectory(double speed = 60, double acc = 80);  // Create trajectory to home position

private:
    std::unique_ptr<c2::CodroidApi> api_;
    bool connected_;
};

#endif // ARM_CONTROLLER_H