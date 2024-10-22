// std
#include <algorithm>
#include <cmath>
#include <execution>
#include <vector>
#include <iostream>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// project
#include "green_detector/green_detector.hpp"
#include "green_detector/types.hpp"
#include "rm_utils/common.hpp"

namespace fyt::auto_aim {
Detector::Detector(const int &bin_thres, const int &color_diff_thr)
: binary_thres(bin_thres), color_diff_thres(color_diff_thr) {}

std::vector<Green> Detector::detect(const cv::Mat &input) noexcept {
    // 1. Preprocess the image
    binary_img = preprocessImage(input);
    // 2. Find green light
    greens     = findGreenLight(input,binary_img);

    return greens;
}

cv::Mat Detector::preprocessImage(const cv::Mat &rgb_img) noexcept {
    cv::cvtColor(rgb_img, gray_img_, cv::COLOR_RGB2GRAY);

    cv::Mat binary_img;
    cv::threshold(gray_img_, binary_img, binary_thres, 255, cv::THRESH_BINARY);

    return binary_img;
}

std::vector<Green> Detector::findGreenLight(const cv::Mat &rgb_img,const cv::Mat &binary_img) noexcept {

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_img,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE,cv::Point());

    std::vector<Green> greens_;
    for(auto contour : contours){
        if(contour.size() < 10) continue;
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);

        int sum_r = 0, sum_g = 0, sum_b = 0;
        for (const auto &point : contour) {
            sum_r += rgb_img.at<cv::Vec3b>(point.y, point.x)[0];
            sum_g += rgb_img.at<cv::Vec3b>(point.y, point.x)[1];
            sum_b += rgb_img.at<cv::Vec3b>(point.y, point.x)[2];
        } 

        // std::cout<<"g-b:"<<std::abs(sum_g - sum_b) / static_cast<int>(contour.size())<<std::endl;
        // std::cout<<"g-r:"<<std::abs(sum_g - sum_r) / static_cast<int>(contour.size()) <<std::endl;
        if ( (std::abs(sum_g - sum_b) / static_cast<int>(contour.size()) > color_diff_thres) && 
             (std::abs(sum_g - sum_r) / static_cast<int>(contour.size()) > color_diff_thres) ) {
            Green green_(center,radius);
            greens_.push_back(green_);
        }
    }
    return greens_;
}

void Detector::drawResults(cv::Mat &img) const noexcept {
    for(auto green_ : greens){
        cv::line(img, green_.center , green_.center, cv::Scalar(0, 0, 255), 2);
        cv::circle(img, green_.center, green_.radius, cv::Scalar(255,0,0), 2);
    }
}


}//namespace fyt::auto_aim