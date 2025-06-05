// #include "ArmController.h"
// #include <iostream>

// ArmController::ArmController(std::shared_ptr<SDK> sdk) : sdk_(sdk) {}

// bool ArmController::moveToHome() {
//     // 示例值
//     std::vector<double> homePose = {0, -90, 90, 0, 90, 0};
//     return sdk_->moveJoints(homePose);
// }

// bool ArmController::openGripper() {
//     return sdk_->controlGripper(true);
// }

// bool ArmController::closeGripper() {
//     return sdk_->controlGripper(false);
// }

// bool ArmController::executeTrajectory(const std::vector<std::vector<double>>& pathPoints) {
//     for (const auto& pt : pathPoints) {
//         if (!sdk_->moveCartesian(pt)) {
//             std::cout << "[执行中断] 点位不可达或受限" << std::endl;
//             return false;
//         }
//     }
//     return true;
// }

// std::vector<std::vector<double>> ArmController::planSafePath(const std::vector<double>& targetPose) {
//     // 伪代码：实际应接入 RRT*/A* 等避障算法
//     return {targetPose};
// }

#include <iostream>
#include <vector>
#include "CodroidApi.h"
#include "ArmController.h"

static void printResponse(const c2::Response& res) {
    std::cout << "code: " << (int)res.code << std::endl;
    std::cout << "msg: " << res.msg << std::endl;
    std::cout << "data: " << res.data.dump(4) << std::endl;
}

int main() {
    std::string path = "Robot/Control/state";
    json        param = { path };
    SetConsoleOutputCP(CP_UTF8);
    
    c2::CodroidApi api("192.168.101.100", "9000");
    c2::Response   res;

    double p1[6] = {493, 83, 440, 180, 0, -90};
    double p2[6] = {568, 190, 440, -180, 0, -90};
    double p3[6] = {478, 265, 440, 180, 0, -90};

    c2::Speed speed{90, 100, 90};
    c2::Acc acc{180, 100, 180};

    //while (true) {
    //    res = api.getRobotState();
    //    printResponse(res);
    //}

#if 0
    res = api.getRobotState();
    printResponse(res);
#endif

#if 0
    res = api.getPackPosition();
    printResponse(res);
#endif

#if 0
    res = api.goHome();
    printResponse(res);
#endif

    //std::thread([&]() {
    //    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //    auto res1 = api.stopMov();
    //    std::cout << "----stopMov----" << std::endl;
    //    printResponse(res1);
    //}).detach();

#if 0
    double p11[6] = {60, 0, 90, 55, 0, -90};
    api.movJNoResult(p11, 90);
#endif

#if 0
    c2::Point point1;
    point1.type = c2::PointType::Cart;
    point1.cpos.x = 455;
    point1.cpos.y = -114;
    point1.cpos.z = 372;
    point1.cpos.a = 172;
    point1.cpos.b = 1;
    point1.cpos.c = -91;

    res = api.movJ(point1);
    std::cout << "----movJ----" << std::endl;
    printResponse(res);
#endif

#if 0
    c2::Point point1;
    point1.type = c2::PointType::Joint;
    point1.apos.jntPos[0] = 3;
    point1.apos.jntPos[1] = 12;
    point1.apos.jntPos[2] = 85;
    point1.apos.jntPos[3] = 0;
    point1.apos.jntPos[4] = 88;
    point1.apos.jntPos[5] = 4;
    res = api.movL(point1, speed);
    std::cout << "----movL----" << std::endl;
    printResponse(res);
#endif

#if 0
    c2::Point point3;
    point3.type           = c2::PointType::Joint;
    point3.apos.jntPos[0] = -9.9;
    point3.apos.jntPos[1] = 9.5;
    point3.apos.jntPos[2] = 87.5;
    point3.apos.jntPos[3] = -0.9;
    point3.apos.jntPos[4] = 90.2;
    point3.apos.jntPos[5] = -9;

    c2::Point point4;
    point4.type           = c2::PointType::Joint;
    point4.apos.jntPos[0] = 7.1;
    point4.apos.jntPos[1] = 3.1;
    point4.apos.jntPos[2] = 95;
    point4.apos.jntPos[3] = 0.4;
    point4.apos.jntPos[4] = 88;
    point4.apos.jntPos[5] = 8;

    res = api.movC(point3, point4, speed);
    std::cout << "----movC----" << std::endl;
    printResponse(res);
#endif

#if 0
    res = api.movC(p3, p2);
    printResponse(res);
#endif

#if 0

    for (int i = 1; i <= 6; i++) {
        c2::JogSpeed jogSpeed;
        c2::Direction direction;

        if (i % 3 == 1) {
            jogSpeed = c2::JogSpeed::Low;
        } else if (i % 3 == 2) {
            jogSpeed = c2::JogSpeed::Mid;
        } else {
            jogSpeed = c2::JogSpeed::High;
        }

        if (i % 2 == 0) {
            direction = c2::Direction::Positive;
        } else {
            direction = c2::Direction::Negative;
        }

        res = api.jointJog(i, jogSpeed, direction);
        printResponse(res);

        for (int j = 0; j < 3; j++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            res = api.keepJog();
            printResponse(res);
        }
    }

    res = api.stopJog();
    printResponse(res);
    
#endif

#if 0

    for (int i = 1; i <= 6; i++) {
        c2::JogSpeed  jogSpeed;
        c2::Direction direction;

        if (i % 3 == 1) {
            jogSpeed = c2::JogSpeed::Low;
        } else if (i % 3 == 2) {
            jogSpeed = c2::JogSpeed::Mid;
        } else {
            jogSpeed = c2::JogSpeed::High;
        }

        if (i % 2 == 0) {
            direction = c2::Direction::Positive;
        } else {
            direction = c2::Direction::Negative;
        }

        res = api.tcpJog(i, jogSpeed, direction);
        printResponse(res);

        for (int j = 0; j < 3; j++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            res = api.keepJog();
            printResponse(res);
        }
    }

    res = api.stopJog();
    printResponse(res);

#endif


#if 0
    res = api.getJointPosition();
    printResponse(res);
    std::vector<double> p14 = res.data;
    std::cout << "vector p14: [";
    for (auto& d : p14) {
        std::cout << d << ", ";
    }
    std::cout << "]" << std::endl;

    double p15[6] = {res.data[0], res.data[1], res.data[2], res.data[3], res.data[4], res.data[5]};
    std::cout << "double p15[6]: [" << p15[0] << ", " << p15[1] << ", " << p15[2] << ", " << p15[3] << ", " << p15[4] << ", "
              << p15[5] << "]" << std::endl;
#endif

#if 0
    res = api.getCartPosition();
    printResponse(res);
    std::vector<double> p16 = res.data;
    std::cout << "vector p16: [";
    for (auto& d : p16) {
        std::cout << d << ", ";
    }
    std::cout << "]" << std::endl;

    double p17[6] = {res.data[0], res.data[1], res.data[2], res.data[3], res.data[4], res.data[5]};
    std::cout << "double p17[6]: [" << p17[0] << ", " << p17[1] << ", " << p17[2] << ", " << p17[3] << ", " << p17[4]
              << ", " << p17[5] << "]" << std::endl;
#endif

#if 0
    c2::CPos cpos;
    cpos.x = 927.5;
    cpos.y = 311.163;
    cpos.z = 931.047;
    cpos.a = -180;
    cpos.b = 35.613;
    cpos.c = -90;

    c2::APos apos;
    apos.jntPos[0] = 0;
    apos.jntPos[1] = 0;
    apos.jntPos[2] = 90;
    apos.jntPos[3] = 0;
    apos.jntPos[4] = 90;
    apos.jntPos[5] = 0;

    res = api.cposToAPos(cpos, apos);
    printResponse(res);
#endif

#if 0
    res = api.sendUserCommand(c2::UserCommand::ToReady);
    printResponse(res);
#endif

#if 0
    res = api.setDO(1,1);
    printResponse(res);
#endif

#if 0
    res = api.getDI(0);
    printResponse(res);
#endif

#if 0
    res = api.setDOGroup(0, 2, 5);
    printResponse(res);
#endif

#if 0
    res = api.getDIGroup(0, 3);
    printResponse(res);
#endif

 #if 0
    c2::UserCoor coor;
    coor.x = 110.2;
    coor.y = 111.2;
    coor.z = 112.2;
    coor.a = 113.2;
    coor.b = 114.2;
    coor.c = 115.2;

    res = api.setCurrentCoor("coor1", coor);
    printResponse(res);
#endif

 #if 0
    c2::Tool tool;
    tool.x = 110.2;
    tool.y = 111.2;
    tool.z = 112.2;
    tool.a = 113.2;
    tool.b = 114.2;
    tool.c = 115.2;

    c2::LoadDyn& dyn = tool.dyn;
    dyn.M      = 2;
    dyn.pos.Mx = 5.3;
    dyn.pos.My = 10.1;
    dyn.pos.Mz = 15.2;

    res = api.setCurrentTool("tool1", tool);
    printResponse(res);
#endif

 #if 0
    c2::LoadDyn dyn;
    dyn.M = 2;
    dyn.pos.Mx = 5.3;
    dyn.pos.My = 10.1;
    dyn.pos.Mz = 15.2;

    res = api.setCurrentPayload("payload1", dyn);
    printResponse(res);
#endif

#if 0

    c2::Zone            zone{99, 200};
    c2::MovCartSegments segments;
    double              pp1[6] = {927.5, 214.5, 900, -180, 0, -90};
    double              pp2[6] = {1173.813, 51.721, 925.973, -168.473, 8.483, -100.169};
    double              pp3[6] = {1077.021, 664.462, 659.728, 170.212, 20.967, -66.149};
    segments.AddMovL(pp1, speed, acc, c2::ZoneType::Relative, zone);
    segments.AddMovL(pp2, speed, acc, c2::ZoneType::Relative, zone);
    segments.AddMovL(pp3, speed, acc, c2::ZoneType::Relative, zone);

    res = api.movCartSegments(segments);
    printResponse(res);
#endif

// 工程运行控制接口测试
 #if 0 

    res = api.runProject("test1");
    printResponse(res);

    std::this_thread::sleep_for(std::chrono::seconds(2));

    res = api.stopProject();
    printResponse(res);

    std::this_thread::sleep_for(std::chrono::seconds(2));

    res = api.runProject("test2");
    printResponse(res);

    std::this_thread::sleep_for(std::chrono::seconds(2));

    res = api.stopProject();
    printResponse(res);

    //res = api.getProjectState();
    //printResponse(res);

    //res = api.runLastProject();
    //printResponse(res);

    //for (int i = 0; i < 3;i++) {
    //    std::this_thread::sleep_for(std::chrono::seconds(2));
    //    res = api.getProjectState();
    //    printResponse(res);
    //}

    //res = api.pauseProject();
    //printResponse(res);

    //std::this_thread::sleep_for(std::chrono::seconds(2));
    //res = api.getProjectState();
    //printResponse(res);

    //res = api.resumeProject();
    //printResponse(res);

    //std::this_thread::sleep_for(std::chrono::seconds(2));
    //res = api.getProjectState();
    //printResponse(res);

    //res = api.stopProject();
    //printResponse(res);

    //std::this_thread::sleep_for(std::chrono::seconds(2));
    //res = api.getProjectState();
    //printResponse(res);
#endif

#if 0
    // 初始化RS485
    res = api.rs485Init("EC2RS485", 115200, 0, 0, 8, 30);
    printResponse(res);

    // 写RS485
    res = api.rs485Write("EC2RS485", {0x01, 0x02, 0x03, 0x04}, 30);
    printResponse(res);

    // RS485清除读缓存
    res = api.rs485FlushReadBuffer("EC2RS485", 30);
    printResponse(res);

    // 读RS485
    res = api.rs485Read("EC2RS485", 9, 30);
    printResponse(res);

#endif

     char a;
     std::cin >> a;

    return 0;
}