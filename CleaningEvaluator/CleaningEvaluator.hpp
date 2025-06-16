#ifndef CLEANING_EVALUATOR_HPP
#define CLEANING_EVALUATOR_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include <random>
#include <filesystem>
#include "nn/onnx_model_base.h"
#include "nn/autobackend.h"
#include "utils/augment.h"
#include "constants.h"
#include "utils/common.h"

namespace fs = std::filesystem;

class CleaningEvaluator {
public:
    CleaningEvaluator();
    ~CleaningEvaluator();
    // Main evaluation function
    double evaluateCleanliness(const cv::Mat& beforeImg, const cv::Mat& afterImg);

    // Helper functions
    cv::Scalar generateRandomColor(int numChannels);
    std::vector<cv::Scalar> generateRandomColors(int class_names_num, int numChannels);
    void plot_masks(cv::Mat img, std::vector<YoloResults>& result, std::vector<cv::Scalar> color,
                   std::unordered_map<int, std::string>& names);
    void plot_keypoints(cv::Mat& image, const std::vector<YoloResults>& results, const cv::Size& shape);
    void plot_results(cv::Mat img, std::vector<YoloResults>& results,
                     std::vector<cv::Scalar> color, std::unordered_map<int, std::string>& names,
                     const cv::Size& shape);
private:
    // Constants and member variables
    std::vector<std::vector<int>> skeleton;
    std::vector<cv::Scalar> posePalette;
    std::vector<int> limbColorIndices;
    std::vector<int> kptColorIndices;
};

#endif // CLEANING_EVALUATOR_HPP
