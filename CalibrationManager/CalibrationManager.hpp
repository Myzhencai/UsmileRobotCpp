#pragma once

class RobotSDK;  // Forward declaration
class Camera;    // Forward declaration

class CalibrationManager {
public:
    CalibrationManager(RobotSDK* robot, Camera* camera);
    
    bool performEndToolCalibration();
    bool performHandEyeCalibration();

private:
    RobotSDK* robot_;
    Camera* camera_;
}; 