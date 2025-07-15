#include <RobotUtilities/spatial_utilities.h>
#include <force_control/admittance_controller.h>
#include <force_control/config_deserialize.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

int main() {
  // load config
  AdmittanceController::AdmittanceControllerConfig admittance_config;
  admittance_config.dt = 0.002;
  admittance_config.log_to_file = false;
  admittance_config.log_file_path = "./admittance_controller.log";
  admittance_config.alert_overrun = false;
  admittance_config.compliance6d.stiffness = (RUT::Matrix6d() << 10, 0, 0, 0, 0, 0,
                                                                 0, 10, 0, 0, 0, 0,
                                                                 0, 0, 10, 0, 0, 0,
                                                                 0, 0, 0, 1, 0, 0,
                                                                 0, 0, 0, 0, 1, 0,
                                                                 0, 0, 0, 0, 0, 1).finished();
  admittance_config.compliance6d.damping = (RUT::Matrix6d() << 2, 0, 0, 0, 0, 0,
                                                                0, 2, 0, 0, 0, 0,
                                                                0, 0, 2, 0, 0, 0,
                                                                0, 0, 0, 0.2, 0, 0,
                                                                0, 0, 0, 0, 0.2, 0,
                                                                0, 0, 0, 0, 0, 0.2).finished();
  admittance_config.compliance6d.inertia = (RUT::Matrix6d() << 2, 0, 0, 0, 0, 0,
                                                                0, 2, 0, 0, 0, 0,
                                                                0, 0, 2, 0, 0, 0,
                                                                0, 0, 0, 0.005, 0, 0,
                                                                0, 0, 0, 0, 0.005, 0,
                                                                0, 0, 0, 0, 0, 0.005).finished();
  admittance_config.compliance6d.stiction = (RUT::Vector6d() << 0, 0, 0, 0, 0, 0).finished();
  admittance_config.max_spring_force_magnitude = 3;
  admittance_config.max_spring_torque_magnitude = 0.5;
  admittance_config.direct_force_control_gains.P_trans = 0.1;
  admittance_config.direct_force_control_gains.I_trans = 0.01;
  admittance_config.direct_force_control_gains.D_trans = 0.01;
  admittance_config.direct_force_control_gains.P_rot = 0;
  admittance_config.direct_force_control_gains.I_rot = 0;
  admittance_config.direct_force_control_gains.D_rot = 0;
  admittance_config.direct_force_control_I_limit = (RUT::Vector6d() << 0, 0, 0, 0, 0, 0).finished();
  // const std::string CONFIG_PATH = "D:/Usmiles/YamlRobot/shoudong.yaml";
  // YAML::Node config{};
  // YAML::Node root = YAML::LoadFile("D:/Usmiles/YamlRobot/shoudong.yaml");
  // YAML::Node points = root["Points"];
  // std::cout << "Points node: " << points << std::endl;
  // try {
  //   std::cout << "Loading config file: " << CONFIG_PATH << std::endl;
  //   config = YAML::LoadFile(CONFIG_PATH);
  //   std::cout << "Config file loaded successfully" << std::endl;
    // deserialize(config["admittance_controller"], admittance_config);
  //   std::cout << "Config deserialized successfully" << std::endl;
  // } catch (const std::exception& e) {
  //   std::cerr << "Failed to load the config file: " << e.what() << std::endl;
  //   std::cerr << "Current working directory might be incorrect" << std::endl;
  //   return -1;
  // }

  AdmittanceController controller;

  RUT::Timer timer;
  RUT::TimePoint time0 = timer.tic();
  RUT::Vector7d pose, pose_ref, pose_cmd;
  RUT::Vector6d wrench, wrench_WTr;

  controller.init(time0, admittance_config, pose);

  // // Regular admittance control, all 6 axes are force dimensions:
  RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
  int n_af = 6;
  controller.setForceControlledAxis(Tr, n_af);

  // HFVC, compliant translational motion, rigid rotation motion
  // RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
  // int n_af = 3;
  // controller.setForceControlledAxis(Tr, n_af);

  // HFVC, compliant rotational motion, rigid translational motion
  // RUT::Matrix6d Tr;
  // Tr << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0,
  //     0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
  // int n_af = 3;

  // controller.setForceControlledAxis(Tr, n_af);

  // n_af = 0 disables compliance. All axes uses rigid motion
  // RUT::Matrix6d Tr = RUT::Matrix6d::Identity();
  // int n_af = 0;
  // controller.setForceControlledAxis(Tr, n_af);

  pose_ref = pose;
  wrench_WTr.setZero();
  wrench_WTr << 5.0, 0.8, 0.3, 0.2, 0.4, 0.15;

  timer.tic();

  while (true) {
    // Update robot status
    pose << 9.97366  , 27.9772 ,  59.9795 , 0.360342 , 0.438611, 0.0298455 , 0.822729;  // 位置和四元数
    wrench << 8.16, 0.8, 0.3, 0.2, 0.4, 0.15;     // 力和力矩
    controller.setRobotStatus(pose, wrench);

    // Update robot reference
    controller.setRobotReference(pose_ref, wrench_WTr);

    // Compute the control output
    controller.step(pose_cmd);

    // send action to robot
    std::cout << "pose_cmd: " << pose_cmd.transpose() << std::endl;

    // sleep till next iteration
    // spin();
  }
}