#include "ForceMonitor.h"
#include "kw-lib-all.h"
#include <vector>
#include <cmath>

/**
 * @brief 构造函数实现
 * @param forcemode 力传感器工作模式 (0:串口, 1:UDP, 2:TCP)
 * @param forcelimit 力阈值限制值
 * @note 初始化力传感器监控器，设置工作模式和力阈值
 */
ForceMonitor::ForceMonitor(int forcemode, double forcelimit)
    : mode(forcemode), forceThreshold(forcelimit)
{
    std::shared_ptr<kw::IOControl> control;
    
    // 根据模式创建不同的控制器
    switch (forcemode) {
        case 0: { // 串口模式
            kw::SerialControlCreator uc;
#ifdef WIN32
            uc.serialPortName = "COM6";
#else
            uc.serialPortName = "/dev/ttyUSB0";
#endif
            uc.baudRate = 460800;
            control = uc.createIOControler();
            break;
        }
        case 1: { // UDP模式
            kw::UdpControlCreator uc;
            uc.sensorIp = "192.168.1.101";
            uc.localIp = "192.168.1.100";
            uc.localPort = 8886;
            control = uc.createIOControler();
            break;
        }
        case 2: { // TCP模式
            std::cout << "在开发状态请选用其他模式" << std::endl;
            throw std::runtime_error("TCP mode not implemented yet");
            break;
        }
        default:
            throw std::runtime_error("Unsupported force sensor mode");
    }

    kw::HeadTailProtocolCreator htc;
    auto proto = htc.createProtocol();

    kw::SensorControlCreator scc;
    scc.ioCtrl = control;
    scc.proto = proto;
    sensor = scc.createSensorControl();
}

/**
 * @brief 读取当前力传感器数据
 * @return 返回包含6个分量的力数据向量，分别对应x,y,z方向的力和力矩
 * @note 如果读取失败或传感器未启动，返回空向量
 */
std::vector<double> ForceMonitor::readForce()
{
    float force[6];
    std::vector<double> result;

    // 检查传感器状态
    if (!sensor) {
        std::cerr << "Error: Force sensor not initialized" << std::endl;
        return result;
    }

    // 启动数据采集
    int startStatus = sensor->StartCapture();
    if (startStatus != 0) {
        std::cerr << "Error: Failed to start force capture, status code: " << startStatus << std::endl;
        return result;
    }

    // 调用传感器接口获取力数据，返回28表示成功
    int status = sensor->GetCurrentForceData(force);
    if (status == 28)
    {
        // 将float数组转换为vector<double>类型
        result.assign(force, force + 6);
    }
    else if (status == 1) {
        std::cerr << "Warning: Lost 1 byte of force data" << std::endl;
    }
    else if (status == 2) {
        std::cerr << "Warning: Lost 2 bytes of force data" << std::endl;
    }
    else {
        std::cerr << "Error: Failed to read force data, status code: " << status << std::endl;
    }

    // 停止数据采集
    int stopStatus = sensor->StopCapture();
    if (stopStatus != 0) {
        std::cerr << "Warning: Failed to stop force capture, status code: " << stopStatus << std::endl;
    }

    return result;
}

/**
 * @brief 检查是否超过力阈值
 * @return 如果任一方向的力超过阈值返回true，否则返回false
 * @note 分别检查6个分量（3个方向的力和力矩），任一超过阈值即认为超载
 */
bool ForceMonitor::isOverloaded()
{
    auto force = readForce();
    if (force.empty())
        return false;

    // 遍历每个力分量，判断是否有超过阈值的情况
    for (const auto &f : force)
    {
        if (std::abs(f) > forceThreshold)
        {
            return true; // 只要有一个分量超过阈值即认为超载
        }
    }
    return false;
}

/**
 * @brief 根据力数据调整轨迹
 * @param trajectory 原始轨迹数据
 * @return 调整后的轨迹数据
 * @note 当前版本直接返回原始轨迹，待实现根据力数据调整轨迹的功能
 */
std::vector<double> ForceMonitor::adjustTrajectory(const std::vector<double> &trajectory)
{
    // TODO: 根据力数据微调路径，需要考虑力反馈进行轨迹补偿
    return trajectory;
}

int main()
{
    // 创建力监控器实例，设置工作模式为0，力阈值为100N
    ForceMonitor monitor(0, 100.0);

    // 主循环
    while (true)
    {
        // 读取当前力数据
        auto force = monitor.readForce();
        
        // 检查是否超载
        if (monitor.isOverloaded())
        {
            std::cout << "Warning: Force threshold exceeded!" << std::endl;
        }

        // 显示当前力数据
        if (!force.empty())
        {
            std::cout << "Current force: ";
            for (const auto &f : force)
            {
                std::cout << f << " ";
            }
            std::cout << std::endl;
        }

        // 添加适当的延时
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
