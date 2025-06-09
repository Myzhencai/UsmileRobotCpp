#include "ArmController.h"
#include <iostream>

/**
 * @brief 构造函数，初始化机器人控制器
 * @param ip 机器人IP地址，默认为"192.168.101.100"
 * @param port 机器人端口号，默认为"9000"
 */
ArmController::ArmController(const std::string& ip, const std::string& port) {
    api_ = std::make_unique<c2::CodroidApi>();
    connected_ = false;
}

/**
 * @brief 析构函数，确保断开连接
 */
ArmController::~ArmController() {
    if (connected_) {
        disconnect();
    }
}

/**
 * @brief 连接到机器人
 * @return 连接是否成功
 */
bool ArmController::connect() {
    if (!api_) {
        std::cerr << "api_ is not initialized." << std::endl;
        return false;
    }
    auto res = api_->connect();
    connected_ = (res.code == c2::ResponseCode::Success);
    return connected_;
}

/**
 * @brief 断开与机器人的连接
 * @return 断开连接是否成功
 */
bool ArmController::disconnect() {
    if (!api_) {
        std::cerr << "api_ is not initialized." << std::endl;
        return false;
    }
    auto res = api_->disconnect();
    connected_ = false;
    return res.code == c2::ResponseCode::Success;
}

/**
 * @brief 检查是否已连接到机器人
 * @return 连接状态
 */
bool ArmController::isConnected() const {
    return connected_;
}



/**
 * @brief 打开夹爪
 * @param port 控制夹爪的数字输出端口号
 * @return 操作是否成功
 */
bool ArmController::openGripper(int port) {
    if (!api_) {
        std::cerr << "api_ is not initialized." << std::endl;
        return false;
    }
    
    auto res = api_->setDO(port, 1); // 设置数字输出为高电平，打开夹爪
    return res.code == c2::ResponseCode::Success;
}

/**
 * @brief 关闭夹爪
 * @param port 控制夹爪的数字输出端口号
 * @return 操作是否成功
 */
bool ArmController::closeGripper(int port) {
    if (!api_) {
        std::cerr << "api_ is not initialized." << std::endl;
        return false;
    }
    
    auto res = api_->setDO(port, 0); // 设置数字输出为低电平，关闭夹爪
    return res.code == c2::ResponseCode::Success;
}

/**
 * @brief 检查夹爪状态
 * @param port 控制夹爪的数字输出端口号
 * @return 夹爪是否处于打开状态
 */
bool ArmController::isGripperOpen(int port) const {
    if (!api_) {
        std::cerr << "api_ is not initialized." << std::endl;
        return false;
    }
    
    auto res = api_->getDI(port); // 读取数字输入状态
    if (res.code == c2::ResponseCode::Success) {
        return res.data.get<int>() == 1; // 如果输入为高电平，则夹爪处于打开状态
    }
    return false;
}

/**
 * @brief 执行笛卡尔轨迹段
 * @param segments 需要执行的笛卡尔轨迹段集合
 * @return 操作是否成功
 */
bool ArmController::executeTrajectory(const MovCartSegments& segments) {
    if (!api_) {
        std::cerr << "api_ is not initialized." << std::endl;
        return false;
    }
    auto res = api_->movCartSegments(segments);
    if (res.code != c2::ResponseCode::Success) {
        std::cout << "[执行中断] movCartSegments 执行失败: " << res.msg << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 设置Home位置
 * @param position 包含6个关节角度的vector
 * @return 设置是否成功
 */
bool ArmController::setHomePosition(const std::vector<double>& position) {
    // 检查api_是否已初始化
    if (!api_) {
        std::cerr << "api_ is not initialized." << std::endl;
        return false;
    }
    
    // 检查输入的关节角度数量是否为6
    if (position.size() != 6) {
        std::cerr << "Home position must have exactly 6 joint angles." << std::endl;
        return false;
    }

    // 拷贝vector到数组
    double pos[6];
    std::copy(position.begin(), position.end(), pos);

    // 调用API设置Home位置
    api_->setHomePosition(pos);
    return true;
}

/**
 * @brief 让机械臂回到Home位置
 * 
 * 调用底层API的goHome方法，使机械臂以指定的速度和加速度运动到Home位置。
 * 
 * @param speed 运动速度（单位：度/秒），默认为60
 * @param acc 运动加速度（单位：度/秒^2），默认为80
 * @return 操作是否成功，true表示成功，false表示失败
 */
bool ArmController::goHome(double speed, double acc) {
    // 检查api_是否已初始化
    if (!api_) {
        std::cerr << "api_ is not initialized." << std::endl;
        return false;
    }

    // 调用API的goHome方法
    auto res = api_->goHome(speed, acc);
    if (res.code != c2::ResponseCode::Success) {
        std::cout << "[执行中断] goHome 执行失败: " << res.msg << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 创建一条回到Home（打包）位的轨迹段
 * 
 * 该函数用于生成一条机器人回到Home（打包）位的轨迹。首先尝试从API获取当前设置的打包位（Home位）关节角度，
 * 如果获取失败，则使用默认的Home位关节角度（[0, 0, 90, 0, 90, 0]）。然后将该点作为目标点，构造一个
 * MovL（直线插补）类型的轨迹段，设置速度、加速度和过渡区类型，最后将该轨迹段加入到轨迹集合中并返回。
 * 
 * @param speed 运动速度（关节空间）
 * @param acc 运动加速度（关节空间）
 * @return MovCartSegments 包含一条回Home位的轨迹段集合
 */
MovCartSegments ArmController::createHomeTrajectory(double speed, double acc) {
    MovCartSegments segments; // 轨迹段集合
    Point point;              // 目标点
    point.type = PointType::Joint; // 目标点类型为关节空间

    // 先把packposition设置为homeposition的位置
    // 获取当前home position
    auto resHome = api_->getHomePosition();
    double homePos[6] = {0, 0, 90, 0, 90, 0};
    if (resHome.code == c2::ResponseCode::Success) {
        auto posVec = resHome.data.get<std::vector<double>>();
        for (int i = 0; i < 6; i++) {
            homePos[i] = posVec[i];
        }
    }
    api_->setPackPosition(homePos);

    // 尝试获取当前设置的打包位（Home位）关节角度
    auto res = api_->getPackPosition();
    if (res.code == c2::ResponseCode::Success) {
        // 获取成功，使用返回的关节角度
        auto posVec = res.data.get<std::vector<double>>();
        for (int i = 0; i < 6; i++) {
            point.apos.jntPos[i] = posVec[i];
        }
    } else {
        // 获取失败，使用默认的Home位关节角度
        point.apos.jntPos[0] = 0;
        point.apos.jntPos[1] = 0;
        point.apos.jntPos[2] = 90;
        point.apos.jntPos[3] = 0;
        point.apos.jntPos[4] = 90;
        point.apos.jntPos[5] = 0;
    }


    // 这里需要添加路径规划的内容修改
    // 构造一个轨迹段，类型为MovL（直线插补），目标点为Home位
    MovCartSegment segment;
    segment.type = MovType::MovL;           // 运动类型：直线插补
    segment.targetPosition = point;         // 目标点
    segment.speed.joint = speed;            // 关节速度
    segment.acc.joint = acc;                // 关节加速度
    segment.zoneType = ZoneType::Fine;      // 过渡区类型：精确停止
    segment.zone.per = 0;                   // 过渡区参数
    segment.zone.dis = 0;                   // 过渡区参数

    // 将轨迹段加入集合
    segments.segments.push_back(segment);
    return segments;
}

// std::vector<std::vector<double>> ArmController::planSafePath(const std::vector<double>& targetPose) {
//     // 伪代码：实际应接入 RRT*/A* 等避障算法
//     return {targetPose};
// }