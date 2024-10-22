// std
#include <cmath>
#include <string>
#include <vector>
// third party 
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
// project
#include "green_detector/types.hpp"

#ifndef GREEN_DETECTOR_DETECTOR_HPP_
#define GREEN_DETECTOR_DETECTOR_HPP_

namespace fyt::auto_aim {
class Detector {
public:

    Detector(const int &bin_thres, const int &color_diff_thr);

    std::vector<Green> detect(const cv::Mat &input) noexcept;

    cv::Mat preprocessImage(const cv::Mat &input) noexcept;

    std::vector<Green> findGreenLight(const cv::Mat &rgb_img,const cv::Mat &binary_img) noexcept;

    // For debug usage
    void drawResults(cv::Mat &img) const noexcept;

    // Parameters
    int binary_thres;
    int color_diff_thres;

    // Debug msgs
    cv::Mat binary_img;

private:

    cv::Mat gray_img_;

    std::vector<Green> greens;

};

}//namespace fyt::auto_aim 

#endif