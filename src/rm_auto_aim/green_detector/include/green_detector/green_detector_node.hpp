// ros2
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// std
#include <memory>
#include <string>
#include <vector>
// project
#include "green_detector/green_detector.hpp"
#include "rm_interfaces/msg/green.hpp"
#include "rm_utils/math/pnp_solver.hpp"

#ifndef GREEN_DETECTOR_DETECTOR_NODE_HPP_
#define GREEN_DETECTOR_DETECTOR_NODE_HPP_

namespace fyt::auto_aim{

class GreenDetectorNode : public rclcpp::Node {
public:
    GreenDetectorNode(const rclcpp::NodeOptions &options);

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
    std::unique_ptr<Detector> initDetector();
    std::vector<Green> detectGreen(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

    void createDebugPublishers() noexcept;    
    void destroyDebugPublishers() noexcept;   

    // Detected armors publisher
    rm_interfaces::msg::Green green_msg_;
    rclcpp::Publisher<rm_interfaces::msg::Green>::SharedPtr green_pub_;

    // Visualization marker publisher
    // visualization_msgs::msg::Marker armor_marker_;
    // visualization_msgs::msg::Marker text_marker_;
    // visualization_msgs::msg::MarkerArray marker_array_;
    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // Camera info part
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    cv::Point2f cam_center_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
    std::unique_ptr<PnPSolver> pnp_solver_;

    // Image subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

    // Armor Detector
    std::unique_ptr<Detector> detector_;

    // ReceiveData subscripiton
    std::string odom_frame_;
    Eigen::Matrix3d imu_to_camera_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    geometry_msgs::msg::TransformStamped odom_to_gimbal;

    // Dynamic Parameter
    rcl_interfaces::msg::SetParametersResult onSetParameters(
        std::vector<rclcpp::Parameter> parameters);
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

    // Debug information
    bool debug_;
    std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
    image_transport::Publisher binary_img_pub_;
    image_transport::Publisher result_img_pub_;
};


}// namespace fyt::auto_aim

#endif