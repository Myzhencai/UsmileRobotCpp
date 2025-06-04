#include <vector>
#include <cmath>

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
    ForceMonitor(ForceSensor* forceSensor) 
        : sensor(forceSensor), forceThreshold(5.0) {}

    std::vector<double> readForce() {
        return sensor->getForce();
    }

    bool isOverloaded() {
        auto force = readForce();
        for (const auto& f : force) {
            if (std::abs(f) > forceThreshold) {
                return true;
            }
        }
        return false;
    }

    std::vector<double> adjustTrajectory(const std::vector<double>& trajectory) {
        // 根据力数据微调路径
        return trajectory;
    }
}; 