#include "CleaningEvaluator.hpp"
#include <sstream>
#include <iomanip>

CleaningEvaluator::CleaningEvaluator() {
    // Initialize skeleton
    skeleton = {{16, 14}, {14, 12}, {17, 15}, {15, 13}, {12, 13}, {6, 12}, {7, 13}, {6, 7},
                {6, 8}, {7, 9}, {8, 10}, {9, 11}, {2, 3}, {1, 2}, {1, 3}, {2, 4}, {3, 5}, {4, 6}, {5, 7}};

    // Initialize pose palette
    posePalette = {
        cv::Scalar(255, 128, 0), cv::Scalar(255, 153, 51), cv::Scalar(255, 178, 102), cv::Scalar(230, 230, 0), cv::Scalar(255, 153, 255),
        cv::Scalar(153, 204, 255), cv::Scalar(255, 102, 255), cv::Scalar(255, 51, 255), cv::Scalar(102, 178, 255), cv::Scalar(51, 153, 255),
        cv::Scalar(255, 153, 153), cv::Scalar(255, 102, 102), cv::Scalar(255, 51, 51), cv::Scalar(153, 255, 153), cv::Scalar(102, 255, 102),
        cv::Scalar(51, 255, 51), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0), cv::Scalar(255, 255, 255)
    };

    // Initialize color indices
    limbColorIndices = {9, 9, 9, 9, 7, 7, 7, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16, 16};
    kptColorIndices = {16, 16, 16, 16, 16, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 9};
}

CleaningEvaluator::~CleaningEvaluator() {
    // Cleanup if needed
}

double CleaningEvaluator::evaluateCleanliness(const cv::Mat& beforeImg, const cv::Mat& afterImg){
    // TODO: Implement the actual cleaning evaluation logic
    // This should compare the before and after images and return a cleanliness score
    return 0.0;
}

cv::Scalar CleaningEvaluator::generateRandomColor(int numChannels) {
    if (numChannels < 1 || numChannels > 3) {
        throw std::invalid_argument("Invalid number of channels. Must be between 1 and 3.");
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, 255);

    cv::Scalar color;
    for (int i = 0; i < numChannels; i++) {
        color[i] = dis(gen);
    }

    return color;
}

std::vector<cv::Scalar> CleaningEvaluator::generateRandomColors(int class_names_num, int numChannels) {
    std::vector<cv::Scalar> colors;
    for (int i = 0; i < class_names_num; i++) {
        colors.push_back(generateRandomColor(numChannels));
    }
    return colors;
}

void CleaningEvaluator::plot_masks(cv::Mat img, std::vector<YoloResults>& result, std::vector<cv::Scalar> color,
                                 std::unordered_map<int, std::string>& names) {
    cv::Mat mask = img.clone();
    for (int i = 0; i < result.size(); i++) {
        float left = result[i].bbox.x;
        float top = result[i].bbox.y;
        int& class_idx = result[i].class_idx;
        rectangle(img, result[i].bbox, color[result[i].class_idx], 2);

        std::string class_name;
        auto it = names.find(class_idx);
        if (it != names.end()) {
            class_name = it->second;
        } else {
            std::cerr << "Warning: class_idx not found in names for class_idx = " << class_idx << std::endl;
            class_name = std::to_string(class_idx);
        }

        if (result[i].mask.rows && result[i].mask.cols > 0) {
            mask(result[i].bbox).setTo(color[result[i].class_idx], result[i].mask);
        }

        std::stringstream labelStream;
        labelStream << class_name << " " << std::fixed << std::setprecision(2) << result[i].conf;
        std::string label = labelStream.str();

        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, nullptr);
        cv::Rect rect_to_fill(left - 1, top - text_size.height - 5, text_size.width + 2, text_size.height + 5);
        cv::Scalar text_color = cv::Scalar(255.0, 255.0, 255.0);
        rectangle(img, rect_to_fill, color[result[i].class_idx], -1);
        putText(img, label, cv::Point(left - 1.5, top - 2.5), cv::FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2);
    }
    addWeighted(img, 0.6, mask, 0.4, 0, img);
}

void CleaningEvaluator::plot_keypoints(cv::Mat& image, const std::vector<YoloResults>& results, const cv::Size& shape) {
    int radius = 5;
    bool drawLines = true;

    if (results.empty()) {
        return;
    }

    std::vector<cv::Scalar> limbColorPalette;
    std::vector<cv::Scalar> kptColorPalette;

    for (int index : limbColorIndices) {
        limbColorPalette.push_back(posePalette[index]);
    }

    for (int index : kptColorIndices) {
        kptColorPalette.push_back(posePalette[index]);
    }

    for (const auto& res: results) {
        auto keypoint = res.keypoints;
        bool isPose = keypoint.size() == 51;
        drawLines &= isPose;

        // Draw points
        for (int i = 0; i < 17; i++) {
            int idx = i * 3;
            int x_coord = static_cast<int>(keypoint[idx]);
            int y_coord = static_cast<int>(keypoint[idx + 1]);

            if (x_coord % shape.width != 0 && y_coord % shape.height != 0) {
                if (keypoint.size() == 3) {
                    float conf = keypoint[2];
                    if (conf < 0.5) {
                        continue;
                    }
                }
                cv::Scalar color_k = isPose ? kptColorPalette[i] : cv::Scalar(0, 0, 255);
                cv::circle(image, cv::Point(x_coord, y_coord), radius, color_k, -1, cv::LINE_AA);
            }
        }

        // Draw lines
        if (drawLines) {
            for (int i = 0; i < skeleton.size(); i++) {
                const std::vector<int>& sk = skeleton[i];
                int idx1 = sk[0] - 1;
                int idx2 = sk[1] - 1;

                int idx1_x_pos = idx1 * 3;
                int idx2_x_pos = idx2 * 3;

                int x1 = static_cast<int>(keypoint[idx1_x_pos]);
                int y1 = static_cast<int>(keypoint[idx1_x_pos + 1]);
                int x2 = static_cast<int>(keypoint[idx2_x_pos]);
                int y2 = static_cast<int>(keypoint[idx2_x_pos + 1]);

                float conf1 = keypoint[idx1_x_pos + 2];
                float conf2 = keypoint[idx2_x_pos + 2];

                if (conf1 < 0.5 || conf2 < 0.5) {
                    continue;
                }

                if (x1 % shape.width == 0 || y1 % shape.height == 0 || x1 < 0 || y1 < 0 ||
                    x2 % shape.width == 0 || y2 % shape.height == 0 || x2 < 0 || y2 < 0) {
                    continue;
                }

                cv::Scalar color_limb = limbColorPalette[i];
                cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), color_limb, 2, cv::LINE_AA);
            }
        }
    }
}

void CleaningEvaluator::plot_results(cv::Mat img, std::vector<YoloResults>& results,
                                   std::vector<cv::Scalar> color, std::unordered_map<int, std::string>& names,
                                   const cv::Size& shape) {
    cv::Mat mask = img.clone();
    int radius = 5;
    bool drawLines = true;
    auto raw_image_shape = img.size();

    std::vector<cv::Scalar> limbColorPalette;
    std::vector<cv::Scalar> kptColorPalette;

    for (int index : limbColorIndices) {
        limbColorPalette.push_back(posePalette[index]);
    }

    for (int index : kptColorIndices) {
        kptColorPalette.push_back(posePalette[index]);
    }

    for (const auto& res : results) {
        float left = res.bbox.x;
        float top = res.bbox.y;

        // Draw bounding box
        rectangle(img, res.bbox, color[res.class_idx], 2);

        // Get class name
        std::string class_name;
        auto it = names.find(res.class_idx);
        if (it != names.end()) {
            class_name = it->second;
        } else {
            std::cerr << "Warning: class_idx not found in names for class_idx = " << res.class_idx << std::endl;
            class_name = std::to_string(res.class_idx);
        }

        // Draw mask if available
        if (res.mask.rows && res.mask.cols > 0) {
            mask(res.bbox).setTo(color[res.class_idx], res.mask);
        }

        // Create and draw label
        std::stringstream labelStream;
        labelStream << class_name << " " << std::fixed << std::setprecision(2) << res.conf;
        std::string label = labelStream.str();

        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, nullptr);
        cv::Rect rect_to_fill(left - 1, top - text_size.height - 5, text_size.width + 2, text_size.height + 5);
        cv::Scalar text_color = cv::Scalar(255.0, 255.0, 255.0);
        rectangle(img, rect_to_fill, color[res.class_idx], -1);
        putText(img, label, cv::Point(left - 1.5, top - 2.5), cv::FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2);

        // Handle keypoints if available
        if (!res.keypoints.empty()) {
            auto keypoint = res.keypoints;
            bool isPose = keypoint.size() == 51;
            drawLines &= isPose;

            // Draw points
            for (int i = 0; i < 17; i++) {
                int idx = i * 3;
                int x_coord = static_cast<int>(keypoint[idx]);
                int y_coord = static_cast<int>(keypoint[idx + 1]);

                if (x_coord % raw_image_shape.width != 0 && y_coord % raw_image_shape.height != 0) {
                    if (keypoint.size() == 3) {
                        float conf = keypoint[2];
                        if (conf < 0.5) {
                            continue;
                        }
                    }
                    cv::Scalar color_k = isPose ? kptColorPalette[i] : cv::Scalar(0, 0, 255);
                    cv::circle(img, cv::Point(x_coord, y_coord), radius, color_k, -1, cv::LINE_AA);
                }
            }

            // Draw lines
            if (drawLines) {
                for (int i = 0; i < skeleton.size(); i++) {
                    const std::vector<int>& sk = skeleton[i];
                    int idx1 = sk[0] - 1;
                    int idx2 = sk[1] - 1;

                    int idx1_x_pos = idx1 * 3;
                    int idx2_x_pos = idx2 * 3;

                    int x1 = static_cast<int>(keypoint[idx1_x_pos]);
                    int y1 = static_cast<int>(keypoint[idx1_x_pos + 1]);
                    int x2 = static_cast<int>(keypoint[idx2_x_pos]);
                    int y2 = static_cast<int>(keypoint[idx2_x_pos + 1]);

                    float conf1 = keypoint[idx1_x_pos + 2];
                    float conf2 = keypoint[idx2_x_pos + 2];

                    if (conf1 < 0.5 || conf2 < 0.5) {
                        continue;
                    }

                    if (x1 % raw_image_shape.width == 0 || y1 % raw_image_shape.height == 0 || x1 < 0 || y1 < 0 ||
                        x2 % raw_image_shape.width == 0 || y2 % raw_image_shape.height == 0 || x2 < 0 || y2 < 0) {
                        continue;
                    }

                    cv::Scalar color_limb = limbColorPalette[i];
                    cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), color_limb, 2, cv::LINE_AA);
                }
            }
        }
    }

    // Combine the image and mask
    addWeighted(img, 0.6, mask, 0.4, 0, img);
} 

int main()
{
    CleaningEvaluator cleanninghandle;
    std::string img_path = "D:/yolov8-onnx-cpp/images/000000000143.jpg";
    //const std::img_path& modelPath = "./checkpoints/yolov8n.onnx"; // detection
    // vs:
    const std::string& modelPath = "D:/yolov8-onnx-cpp/checkpoints/yolov8n-seg.onnx"; // instance segmentation
    // clion:
    // const std::string& modelPath = "D:/yolov8-onnx-cpp/checkpoints/yolov8n-pose.onnx"; // pose

    fs::path imageFilePath(img_path);
    fs::path newFilePath = imageFilePath.stem();
    newFilePath += "-kpt-cpp";
    newFilePath += imageFilePath.extension();
    assert(newFilePath != imageFilePath);
    std::cout << "newFilePath: " << newFilePath << std::endl;

    const std::string& onnx_provider = OnnxProviders::CPU; // "cpu";
    const std::string& onnx_logid = "yolov8_inference2";
    float mask_threshold = 0.5f;  // in python it's 0.5 and you can see that at ultralytics/utils/ops.process_mask line 705 (ultralytics.__version__ == .160)
    float conf_threshold = 0.30f;
    float iou_threshold = 0.45f;  //  0.70f;
	int conversion_code = cv::COLOR_BGR2RGB;
    cv::Mat img = cv::imread(img_path, cv::IMREAD_UNCHANGED);
    if (img.empty()) {
        std::cerr << "Error: Unable to load image" << std::endl;
        return 1;
    }
    AutoBackendOnnx model(modelPath.c_str(), onnx_logid.c_str(), onnx_provider.c_str());
    std::vector<YoloResults> objs = model.predict_once(img, conf_threshold, iou_threshold, mask_threshold, conversion_code);
    std::vector<cv::Scalar> colors = cleanninghandle.generateRandomColors(model.getNc(), model.getCh());
    std::unordered_map<int, std::string> names = model.getNames();

    std::vector<std::vector<float>> keypointsVector;
    for (const YoloResults& result : objs) {
        keypointsVector.push_back(result.keypoints);
    }

    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    cv::Size show_shape = img.size();  // cv::Size(1280, 720); // img.size()
    cleanninghandle.plot_results(img, objs, colors, names, show_shape);
//    plot_masks(img, objs, colors, names);
    cv::imshow("img", img);
    cv::waitKey(0);
    return -1;
}