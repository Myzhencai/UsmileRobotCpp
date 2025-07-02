#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <string>
#include <vector>
#include <memory>
#include "CodroidApi.h"
#include "Define.h"
#include <open3d/Open3D.h>
#include <Eigen/Dense>

// 需要对机械臂设置安全区域避免出错
class ArmController {
public:
    /**
     * @brief 构造函数，初始化机器人控制器
     * @param ip 机器人IP地址，默认为"192.168.101.100"
     * @param port 机器人端口号，默认为"9000"
     */
    ArmController(const std::string& ip = "192.168.101.100", const std::string& port = "9000");

    /**
     * @brief 析构函数，确保断开连接
     */
    ~ArmController();

    /**
     * @brief 检查是否已连接机器人
     * @return true表示已连接，false表示未连接
     */
    bool isconnected();

    /**
     * @brief 断开与机器人的连接
     * @return true表示断开成功，false表示失败
     */
    bool disconnect();

    /**
     * @brief 打开夹爪（暂时不用）
     * @param port 控制夹爪的数字输出端口号，默认1
     * @return true表示操作成功，false表示失败
     */
    bool openGripper(int port = 1);

    /**
     * @brief 关闭夹爪（暂时不用）
     * @param port 控制夹爪的数字输出端口号，默认1
     * @return true表示操作成功，false表示失败
     */
    bool closeGripper(int port = 1);

    /**
     * @brief 检查夹爪是否处于打开状态（暂时不用）
     * @param port 控制夹爪的数字输出端口号，默认1
     * @return true表示夹爪打开，false表示关闭
     */
    bool isGripperOpen(int port = 1) const;

    /**
     * @brief 根据关节空间轨迹段移动机械臂
     * @param segments 关节空间轨迹段集合
     * @return true表示执行成功，false表示失败
     */
    bool executeTrajectory(const c2::MovJointSegments& segments);

    /**
     * @brief 从yaml文件生成关节空间轨迹
     * @param yaml_path yaml文件路径
     * @param speed 关节运动速度（度/秒）
     * @param acc 关节运动加速度（度/秒^2）
     * @return 生成的关节空间轨迹段集合
     */
    c2::MovJointSegments parseYamlToMovJointSegments(const std::string& yaml_path, double speed = 60, double acc = 80);

    /**
     * @brief 设置Home位置（关节角度）
     * @param position 6维关节角度
     * @return true表示设置成功，false表示失败
     */
    bool setHomePosition(const std::vector<double>& position);

    /**
     * @brief 根据点云和法向量生成笛卡尔空间轨迹
     * @param cloud 点云（已在机械臂坐标系下）
     * @param ids 轨迹点在点云中的索引
     * @param speed 末端线速度（mm/s）
     * @param acc 末端线加速度（mm/s^2）
     * @param extend_dist 沿法向量延伸距离（米），默认0.05m
     * @return 生成的笛卡尔空间轨迹段集合
     */
    c2::MovCartSegments createCartTrajectoryFromPointCloud(const open3d::geometry::PointCloud& cloud, const std::vector<int>& ids, double speed = 60, double acc = 80, double extend_dist = 0.05);

    /**
     * @brief 按照生成的MovCartSegments轨迹来运动
     * @param traj 笛卡尔空间轨迹段集合
     * @return true表示执行成功，false表示失败
     */
    bool StartBrushTeethTrajectory(const c2::MovCartSegments& traj);

    /**
     * @brief 执行指定的project
     * @param projectName 工程名称
     * @param taskName 任务名称（可选）
     * @return true表示执行成功，false表示失败
     */
    bool runProject(const std::string& projectName, const std::string& taskName = "");

    /**
     * @brief 获取当前机械臂状态
     * @return 状态码（参考c2::RobotState），失败返回-1
     */
    int getRobotState();

    /**
     * @brief 获取当前机械臂末端的笛卡尔坐标系位姿
     * @param out_pose 输出参数，返回位姿（c2::CPos）
     * @return true表示获取成功，false表示失败
     */
    bool getCartPosition(c2::CPos& out_pose);

    /**
     * @brief 改变机械臂模式（如切换到手动/自动等）
     * @param mode 目标模式（参考c2::UserCommand）
     * @return true表示切换成功，false表示失败
     */
    bool changeRobotMode(c2::UserCommand mode);

private:
    std::unique_ptr<c2::CodroidApi> api_;
    bool connected_;
};

#endif // ARM_CONTROLLER_H