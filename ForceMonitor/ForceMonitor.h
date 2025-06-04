#pragma once

#include <vector>

class ForceSensor {
public:
    virtual std::vector<double> getForce() = 0;
    virtual ~ForceSensor() = default;
};

class ForceMonitor {
private:
    ForceSensor* sensor;
    double forceThreshold;

public:
    ForceMonitor(ForceSensor* forceSensor);
    std::vector<double> readForce();
    bool isOverloaded();
    std::vector<double> adjustTrajectory(const std::vector<double>& trajectory);
}; 