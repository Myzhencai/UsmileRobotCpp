#ifndef SCANNER_H
#define SCANNER_H

#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <array>
#include "TJST3DScannerApi.h"

class Scanner
{
public:
    Scanner(bool is_initialized, int camera_group_id);
    ~Scanner();
    bool initialize(int mode, int dev_num);
    bool enter_scan_mode();
    void uninitialize();
    bool set_scan_parameters(float exposure_time, float trigger_delay, int repeat, int trigger_num);
    bool set_led_light(unsigned char light);
    bool save_pointcloud(const std::string &filename);
    bool load_pointcloud(const std::string &filename);
    bool show_pointcloud(sn3DCore::sn3DRangeData *pointcloud);
    bool scan_single_pointcloud();
    bool merge_scanned_pointclouds(const std::string &output_file);
    void clear_scanned_pointclouds();
    bool capture_image(const std::string &left_image_filename, const std::string &right_image_filename, bool bRGB);
    sn3DCore::sn3DRangeData *transform_pointcloud(
        sn3DCore::sn3DRangeData *pointcloud,
        const std::array<double, 3> &translation,
        const std::array<double, 3> &rotation);

private:
    bool is_initialized_; ///< 扫描仪是否已初始化
    int camera_group_id_;
    std::vector<sn3DCore::sn3DRangeData *> scanned_pointclouds_;
};

#endif // SCANNER_H
