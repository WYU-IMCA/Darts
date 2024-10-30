
#ifndef GREEN_SOLVER_SOLVER_NODE_HPP_
#define GREEN_SOLVER_SOLVER_NODE_HPP_

// ros2
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// std
#include <memory>
#include <string>
#include <vector>
// project
#include "green_solver/green_solver.hpp"
#include "rm_interfaces/msg/green.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"

namespace fyt::auto_aim {
using tf2_filter = tf2_ros::MessageFilter<rm_interfaces::msg::Green>;
class GreenSolverNode : public rclcpp::Node {
public:
  explicit GreenSolverNode(const rclcpp::NodeOptions &options);

private:
  // Green Solver
  std::unique_ptr<Solver> solver_;

  void greenCallback(const rm_interfaces::msg::Green::SharedPtr green_ptr);
  
  rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;

  rclcpp::Subscription<rm_interfaces::msg::Green>::SharedPtr green_sub_;
  
};


}//namespace fyt::auto_aim


#endif