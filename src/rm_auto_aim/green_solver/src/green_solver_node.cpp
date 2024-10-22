#include "green_solver/green_solver_node.hpp"
//std
#include <vector>
#include <memory>

namespace fyt::auto_aim {
GreenSolverNode::GreenSolverNode(const rclcpp::NodeOptions &options)
: Node("green_solver", options), solver_(nullptr) {

  gimbal_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>("green_solver/cmd_gimbal",
                                                                      rclcpp::SensorDataQoS());

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  green_sub_.subscribe(this, "green_detector/green", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_filter>(green_sub_,
                                             *tf2_buffer_,
                                             target_frame_,
                                             10,
                                             this->get_node_logging_interface(),
                                             this->get_node_clock_interface(),
                                             std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when
  // transforms are available
  tf2_filter_->registerCallback(&GreenSolverNode::greenCallback, this);

}

void GreenSolverNode::greenCallback(const rm_interfaces::msg::Green::SharedPtr green_ptr) {

  // Lazy initialize solver owing to weak_from_this() can't be called in constructor
  if (solver_ == nullptr) {
    solver_ = std::make_unique<Solver>(weak_from_this());
    RCLCPP_INFO(rclcpp::get_logger("green_solver"),"CREATED solver");
  }

  // Tranform armor position from image frame to world coordinate
  if(green_ptr->pose.position.x != 0 && green_ptr->pose.position.y != 0 && green_ptr->pose.position.z != 0) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = green_ptr->header;
    ps.pose = green_ptr->pose;
    try {
      green_ptr->pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::TransformException &ex) {
      // RCLCPP_ERROR(rclcpp::get_logger("green_solver"),"Transform error: %s", ex.what());
      return;
    }
  }


  bool tracking = true;
  if(green_ptr->pose.position.x < 0){
    tracking = false;
  }

  // Solve control command    
  rm_interfaces::msg::GimbalCmd control_msg;
  if(tracking){
    control_msg = solver_->solve(*green_ptr);
  }else{
    control_msg.yaw = 0;
  }
  // control_msg.yaw = 25.5;
  gimbal_pub_->publish(control_msg);
}

}//namespace fyt::auto_aim 

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::GreenSolverNode)