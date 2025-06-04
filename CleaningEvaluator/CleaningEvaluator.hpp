#ifndef CLEANING_EVALUATOR_HPP
#define CLEANING_EVALUATOR_HPP

#include <opencv2/opencv.hpp>
#include <string>

class CleaningEvaluator {
public:
    CleaningEvaluator();
    double evaluateCleanliness(const std::string& beforeImgPath, const std::string& afterImgPath);
};

#endif // CLEANING_EVALUATOR_HPP 