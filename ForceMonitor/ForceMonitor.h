#pragma once

#include <vector>
#include <memory>
#include "kw-lib-all.h"

class ForceMonitor {
public:
    ForceMonitor(std::shared_ptr<kw::SensorControl> forceSensor);
    std::vector<double> readForce();
    bool isOverloaded();
    std::vector<double> adjustTrajectory(const std::vector<double>& trajectory);

private:
    std::shared_ptr<kw::SensorControl> sensor;
    double forceThreshold;
}; 