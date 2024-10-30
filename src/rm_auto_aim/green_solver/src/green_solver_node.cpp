#include "green_solver/green_solver_node.hpp"
#include "rm_utils/logger/log.hpp"
//std
#include <vector>
#include <memory>

namespace fyt::auto_aim {
GreenSolverNode::GreenSolverNode(const rclcpp::NodeOptions &options)
: Node("green_solver", options), solver_(nullptr) {

  FYT_REGISTER_LOGGER("serial_driver", "~/fyt2024-log", INFO);

  gimbal_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>("green_solver/cmd_gimbal",
                                                                      rclcpp::SensorDataQoS());

  green_sub_ = this->create_subscription<rm_interfaces::msg::Green>
  ("green_detector/green",rclcpp::SensorDataQoS(),std::bind(&GreenSolverNode::greenCallback, this, std::placeholders::_1));
}

void GreenSolverNode::greenCallback(const rm_interfaces::msg::Green::SharedPtr green_ptr) {

  // Lazy initialize solver owing to weak_from_this() can't be called in constructor
  if (solver_ == nullptr) {
    solver_ = std::make_unique<Solver>(weak_from_this());
    RCLCPP_INFO(rclcpp::get_logger("green_solver"),"CREATED solver");
  }

  bool tracking = true;
  if(green_ptr->pose.position.z < 0){
    tracking = false;
  }
  // Solve control command    
  rm_interfaces::msg::GimbalCmd control_msg;
  if(tracking){
    control_msg = solver_->solve(*green_ptr);
  }else{
    control_msg.yaw = 0;
  }
  gimbal_pub_->publish(control_msg);
}

}//namespace fyt::auto_aim 

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::GreenSolverNode)