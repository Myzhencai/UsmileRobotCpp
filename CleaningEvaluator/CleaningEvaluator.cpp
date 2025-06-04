#include "CleaningEvaluator.hpp"
#include <cmath>

CleaningEvaluator::CleaningEvaluator() {
}

double CleaningEvaluator::evaluateCleanliness(const std::string& beforeImgPath, const std::string& afterImgPath) {
    // 读取图像
    cv::Mat before = cv::imread(beforeImgPath, cv::IMREAD_GRAYSCALE);
    cv::Mat after = cv::imread(afterImgPath, cv::IMREAD_GRAYSCALE);

    if (before.empty() || after.empty()) {
        return -1.0; // 返回错误值
    }

    // 计算差异
    cv::Mat diff;
    cv::absdiff(before, after, diff);

    // 计算平均差异
    cv::Scalar meanDiff = cv::mean(diff);
    double score = 1.0 - meanDiff[0] / 255.0;

    // 四舍五入到3位小数
    return std::round(score * 1000.0) / 1000.0;
} 