#pragma once

#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::ordered_json;

namespace c2 {

enum class ResponseCode : int {
    OK = 0,         // 请求正常
    Timeout,        // 请求超时
    QueueFull,      // 请求队列已满
    NetError,       // 网络异常
    RequestFailed,  // 请求失败
};

struct Response {
    ResponseCode code{ResponseCode::OK};
    std::string  msg;
    json         data;
};

/**
 * 机器人状态.
 */
enum class RobotState : int {
    None    = -1,  // 未知
    Init    = 0,   // 初始化
    StandBy = 1,   // 已下电
    Ready   = 2,   // 手动模式
    Rescue  = 3,   // 救援模式
    Auto    = 4,   // 自动模式
    Error   = 6,   // 机器人出错
};

/**
 * 速度
 */
struct Speed {
    double joint{0};  // 关节速度,适用关节空间运动, 单位：度/秒（deg/s）
    double tcp{0};    // 末端线速度,适用笛卡尔空间运动,单位：毫米/秒（mm/s）
    double ori{0};    // 末端旋转角速度，适用笛卡尔空间运动,单位：度/秒（deg/s）
};

/**
 * 加速度.
 * double joint{0};  // 关节加速度,适用关节空间运动, 单位：度/秒平方（deg/s2）
 * double tcp{0};    // 末端线加速度,适用笛卡尔空间运动,单位：毫米/秒平方（mm/s2）
 * double ori{0};    // 末端旋转角加速度，适用笛卡尔空间运动,单位：度/秒平方（deg/s2）
 */
using Acc = Speed;

enum class UserCommand : int {
    SwitchOn         = 1,    // 上电
    SwitchOff        = 2,    // 下电
    ToReady          = 3,    // 进入手动模式
    ToAuto           = 5,    // 进入自动模式
    AcknowledgeError = 100,  // 清除错误
};

enum class JogSpeed : int {
    Low  = 1,  // 低速
    Mid  = 2,  // 中速
    High = 3,  // 高速
};

enum class Direction : int {
    Positive = 1,   // 正方向
    Negative = -1,  // 负方向
};

struct UserCoor {
    double x;  // 用户坐标系原点相对于世界坐标系在 x 方向的位移偏移量, 单位: mm;
    double y;  // 用户坐标系原点相对于世界坐标系在 y 方向的位移偏移量, 单位: mm;
    double z;  // 用户坐标系原点相对于世界坐标系在 z 方向的位移偏移量, 单位: mm;
    double a;  // 用户坐标系相对于世界坐标系 x 轴旋转的欧拉角, 单位: deg;
    double b;  // 用户坐标系相对于世界坐标系 y 轴旋转的欧拉角, 单位: deg;
    double c;  // 用户坐标系相对于世界坐标系 z 轴旋转的欧拉角, 单位: deg.
};

/**
 * @brief 质心矢量是以安装的工具或负载在坐标系 Otool-XYZ 上的位置设定.
 *
 */
struct CenterPos {
    double Mx{0};  // 安装的工具或装夹的负载的重心 C 在坐标系 Otool-XYZ 的 X 方向上的偏移量, 单位: mm;
    double My{0};  // 安装的工具或装夹的负载的重心 C 在坐标系 Otool-XYZ 的 Y 方向上的偏移量, 单位: mm;
    double Mz{0};  // 安装的工具或装夹的负载的重心 C 在坐标系 Otool-XYZ 的 Z 方向上的偏移量, 单位: mm.
};

/**
 * @brief 该参数是以安装的工具或负载由输出坐标系 Otool-XYZ 决定的惯性张量.
 *const auto&
 */
struct InertiaTensor {
    double Ixx{0};  // 安装的工具或装夹的负载在重心处 X  方向回转的惯量, 单位: kg*mm2;
    double Iyy{0};  // 安装的工具或装夹的负载在重心处 Y  方向回转的惯量, 单位: kg*mm2;
    double Izz{0};  // 安装的工具或装夹的负载在重心处 Z  方向回转的惯量, 单位: kg*mm2;
    double Ixy{0};  // 安装的工具或装夹的负载在重心处 XY 交叉方向的惯量积, 单位: kg*mm2;
    double Ixz{0};  // 安装的工具或装夹的负载在重心处 XZ 交叉方向的惯量积, 单位: kg*mm2;
    double Iyz{0};  // 安装的工具或装夹的负载在重心处 YZ 交叉方向的惯量积, 单位: kg*mm2.
};

/**
 * @brief 用来存储机器人末端工具和负载质量信息参数，用于机器人动力学全模型计算.
 *
 */
struct LoadDyn {
    double        M{0};  // 工具&负载的重量, 单位: kg;
    CenterPos     pos;   // 参见 CenterPos;
    InertiaTensor it;    // 参见 InertiaTensor.
};

struct Tool {
    double x{0};  // TCP相对于法兰坐标系在x方向的位移偏移量, 单位: mm;
    double y{0};  // TCP相对于法兰坐标系在x方向的位移偏移量, 单位: mm;
    double z{0};  // TCP相对于法兰坐标系在z方向的位移偏移量, 单位: mm;
    double a{0};  // TCP相对于法兰坐标系 x 轴旋转的欧拉角, 单位: deg;
    double b{0};  // TCP相对于法兰坐标系 y 轴旋转的欧拉角, 单位: deg;
    double c{0};  // TCP相对于法兰坐标系 z 轴旋转的欧拉角, 单位: deg;

    LoadDyn dyn;
};

/// <summary>
/// 过渡类型
/// </summary>
enum class ZoneType : int {
    Fine,      // 不过渡
    Relative,  // 相对过渡
    Absolute,  // 绝对过渡
};


struct Zone {
    double per = 0;  // 转弯百分比值，过渡类型为相对过渡时生效，适用所有运动类型
    double dis = 0;  // 笛卡尔空间转弯区大小，单位: mm，过渡类型为绝对过渡时生效，不适用MovJ
};

enum class MovType : int {
    MovJ,
    MovL,
    MovC,
    MovCircle,
};

enum class PointType {
    Joint, // 关节位置
    Cart, // 笛卡尔位置
};

struct APos {
    double jntPos[7]{0};  // 机器人 n 关节的位置, 单位: m(直线电机) 或 rad(旋转电机);

    static bool toJson(nlohmann::ordered_json& json, const void* value) {
        const auto& var = *(APos*)value;

        json["jntpos1"] = var.jntPos[0];
        json["jntpos2"] = var.jntPos[1];
        json["jntpos3"] = var.jntPos[2];
        json["jntpos4"] = var.jntPos[3];
        json["jntpos5"] = var.jntPos[4];
        json["jntpos6"] = var.jntPos[5];
        json["jntpos7"] = var.jntPos[6];

        return true;
    }

    static bool fromJson(const nlohmann::ordered_json& json, void* value) {
        auto&& var = *(APos*)value;
        try {
            var.jntPos[0] = json["jntpos1"].get<double>();
            var.jntPos[1] = json["jntpos2"].get<double>();
            var.jntPos[2] = json["jntpos3"].get<double>();
            var.jntPos[3] = json["jntpos4"].get<double>();
            var.jntPos[4] = json["jntpos5"].get<double>();
            var.jntPos[5] = json["jntpos6"].get<double>();

            if (json.contains("jntpos7")) {
                var.jntPos[6] = json["jntpos7"].get<double>();
            } else {
                var.jntPos[6] = 0;
            }


        } catch (...) {
            return false;
        }

        return true;
    }
};

/**
 * @brief 机器人在相同的笛卡尔空间位置下，可以具备多种关节位置组合对应
 *        (机器人逆解的多解).该属性用于定义空间目标点对应的形态配置数据.
 *
 */
struct PosCfg {
    int mode{-1};  // 机器人工作构型选取参数;

    int cf[7]{0};  // 关节 n 号轴角度所在的象限取值;

    static bool toJson(nlohmann::ordered_json& json, const void* value) {
        const auto& var = *(PosCfg*)value;

        json["mode"] = var.mode;
        json["cf1"]  = var.cf[0];
        json["cf2"]  = var.cf[1];
        json["cf3"]  = var.cf[2];
        json["cf4"]  = var.cf[3];
        json["cf5"]  = var.cf[4];
        json["cf6"]  = var.cf[5];
        json["cf7"]  = var.cf[6];

        return true;
    }

    static bool fromJson(const nlohmann::ordered_json& json, void* value) {
        auto&& var = *(PosCfg*)value;
        try {
            var.mode  = json["mode"];
            var.cf[0] = json["cf1"];
            var.cf[1] = json["cf2"];
            var.cf[2] = json["cf3"];
            var.cf[3] = json["cf4"];
            var.cf[4] = json["cf5"];
            var.cf[5] = json["cf6"];

            if (json.contains("cf7")) {
                var.cf[6] = json["cf7"];
            } else {
                var.cf[6] = 0;
            }

        } catch (...) {
            return false;
        }

        return true;
    }
};

/**
 * @brief TCP 点在笛卡尔坐标系下位置.
 *
 */
struct CPos {
    PosCfg cfgData;  // 参见 PosCfg;

    double x{0};  // TCP 点在参考坐标系上 x 方向的坐标, 单位: m;
    double y{0};  // TCP 点在参考坐标系上 y 方向的坐标, 单位: m;
    double z{0};  // TCP 点在参考坐标系上 z 方向的坐标, 单位: m;
    double a{0};  // TCP 点相对于参考坐标系 z 轴旋转的欧拉角, 单位: rad;
    double b{0};  // TCP 点相对于参考坐标系 y′轴旋转的欧拉角, 单位: rad;
    double c{0};  // TCP 点相对于参考坐标系 x″轴旋转的欧拉角, 单位: rad;

    double e{0};  // 7 自由度机器人肘关节 (elbow) 姿态;

    static bool toJson(nlohmann::ordered_json& json, const void* value) {
        const auto& var = *(CPos*)value;

        json["x"] = var.x;
        json["y"] = var.y;
        json["z"] = var.z;
        json["a"] = var.a;
        json["b"] = var.b;
        json["c"] = var.c;
        json["e"] = var.e;

        PosCfg::toJson(json["poscfg"], &var.cfgData);

        return true;
    }

    static bool fromJson(const nlohmann::ordered_json& json, void* value) {
        auto&& var = *(CPos*)value;
        try {
            var.x = json["x"].get<double>();
            var.y = json["y"].get<double>();
            var.z = json["z"].get<double>();
            var.a = json["a"].get<double>();
            var.b = json["b"].get<double>();
            var.c = json["c"].get<double>();
            if (json.contains("e")) {
                var.e = json["e"].get<double>();
            } else {
                var.e = 0;
            }

        } catch (...) {
            return false;
        }
        if (!PosCfg::fromJson(json["poscfg"], &var.cfgData)) {
            return false;
        }
        return true;
    }
};

struct Point {
    PointType type{PointType::Joint};
    APos      apos;
    CPos      cpos;
};


struct MovSegment {
    MovType  type;
    Point    targetPosition;
    Point    middlePosition;
    Speed    speed;
    Acc      acc;
    ZoneType zoneType;
    Zone     zone;
};

struct MovJointSegments {
    std::vector<MovSegment> segments;

    void AddMovJ(Point position, Speed speed, Acc acc, ZoneType zoneType, Zone zone) {
        if (zoneType == ZoneType::Absolute) {
            throw std::string("zoneType can not be ABSOLUTE");
        }

        segments.emplace_back();
        MovSegment& segment = segments.back();

        segment.type = MovType::MovJ;
        segment.targetPosition = position;
        segment.speed    = speed;
        segment.acc      = acc;
        segment.zoneType = zoneType;
        segment.zone     = zone;
    }
};

struct MovCartSegments {
    std::vector<MovSegment> segments;

    void AddMovL(Point position, Speed speed, Acc acc, ZoneType zoneType, Zone zone) {
        segments.emplace_back();
        MovSegment& segment = segments.back();

        segment.type = MovType::MovL;
        segment.targetPosition = position;
        segment.speed    = speed;
        segment.acc      = acc;
        segment.zoneType = zoneType;
        segment.zone     = zone;
    }

    void AddMoC(Point middlePosition, Point targetPosition, Speed speed, Acc acc, ZoneType zoneType, Zone zone) {
        segments.emplace_back();
        MovSegment& segment = segments.back();

        segment.type = MovType::MovC;
        segment.middlePosition = middlePosition;
        segment.targetPosition = targetPosition;
        segment.speed    = speed;
        segment.acc      = acc;
        segment.zoneType = zoneType;
        segment.zone     = zone;
    }

    void AddMoCircle(Point middlePosition, Point targetPosition,
                     Speed    speed,
                     Acc      acc,
                     ZoneType zoneType,
                     Zone     zone) {
        segments.emplace_back();
        MovSegment& segment = segments.back();

        segment.type = MovType::MovCircle;
        segment.middlePosition = middlePosition;
        segment.targetPosition = targetPosition;
        segment.speed    = speed;
        segment.acc      = acc;
        segment.zoneType = zoneType;
        segment.zone     = zone;
    }
};

}  // namespace c2
