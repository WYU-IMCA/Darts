#include "green_solver/green_solver.hpp"
// std
#include <cmath>
#include <cstddef>
#include <stdexcept>
// project
#include "green_solver/green_solver_node.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace fyt::auto_aim {
Solver::Solver(std::weak_ptr<rclcpp::Node> n) : node_(n) {
    auto node = node_.lock();

    node.reset();
}

rm_interfaces::msg::GimbalCmd Solver::solve(const rm_interfaces::msg::Green &green_msg) {
    Eigen::Vector3d p;
    p<<green_msg.pose.position.x,green_msg.pose.position.y,green_msg.pose.position.z;
    
    double green_x = green_msg.pose_pixel.position.x;

    // Initialize gimbal_cmd
    rm_interfaces::msg::GimbalCmd gimbal_cmd; 
    gimbal_cmd.yaw = green_x - 639.979736;
    // gimbal_cmd.yaw = calcYaw(p);
    return gimbal_cmd;
}

double Solver::calcYaw(const Eigen::Vector3d &p) const noexcept {
  // Calculate yaw and pitch
  double yaw = atan2(p.y(), p.x()) * 180 / CV_PI;
  return yaw;
}

} //namespace fyt::auto_aim