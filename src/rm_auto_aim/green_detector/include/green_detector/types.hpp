
// std
#include <algorithm>
#include <numeric>
#include <string>
// 3rd party
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
// project

#ifndef GREEN_DETECTOR_TYPES_HPP_
#define GREEN_DETECTOR_TYPES_HPP_

namespace fyt::auto_aim {

// Armor size, Unit: m
constexpr double GREEN_RADIUS = 17.0 / 1000.0; // 55

struct Green {
  static constexpr const int N_LANDMARKS = 4;
  Green() = default;
  Green(const cv::Point2d center_, const double radius_) {
    center = center_;
    radius = radius_;
    top = cv::Point2f(center.x,center.y-radius);
    bottom = cv::Point2f(center.x,center.y+radius);
    left = cv::Point2f(center.x-radius,center.y);
    right = cv::Point2f(center.x+radius,center.y);
  }  

  // Build the points in the object coordinate system, start from bottom left in
  // clockwise order
  template <typename PointType>
  static inline std::vector<PointType> buildObjectPoints(const double &radius) noexcept {
    return {PointType(0, radius, 0),
            PointType(0, 0, radius),
            PointType(0, -radius, 0),
            PointType(0, 0, -radius)};
  }

  std::vector<cv::Point2f> landmarks() const {
    return {left, top, right, bottom};
  }

  cv::Point2f top, bottom, left, right;
  cv::Point2d center;
  double radius;
};

}  // namespace fyt::auto_aim

#endif

