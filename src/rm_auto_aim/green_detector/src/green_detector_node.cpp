// std
#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <vector>
// ros2
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// third party
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// project 
#include "green_detector/green_detector_node.hpp"

namespace fyt::auto_aim
{
  GreenDetectorNode::GreenDetectorNode(const rclcpp::NodeOptions &options)
      : Node("green_detector", options)
  {
    RCLCPP_INFO(rclcpp::get_logger("green_detector"),"Starting GreenDetectorNode!");

    // Detector
    detector_ = initDetector();

    // Armors Publisher
    green_pub_ = this->create_publisher<rm_interfaces::msg::Green>("green_detector/green",
                                                                     rclcpp::SensorDataQoS());

    odom_frame_ = this->declare_parameter("target_frame", "odom");

    // Debug Publishers
    debug_ = this->declare_parameter("debug", true);

    if (debug_)
    {
      createDebugPublishers();
    }
    // Debug param change moniter
    debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    debug_cb_handle_ = debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter &p) 
    { debug_ = p.as_bool();
      debug_ ? createDebugPublishers() : destroyDebugPublishers(); });

    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
        {
          cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
          cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
          // Setup pnp solver
          pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
          pnp_solver_->setObjectPoints(
              "green", Green::buildObjectPoints<cv::Point3f>(GREEN_RADIUS));
          cam_info_sub_.reset();
        });

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&GreenDetectorNode::imageCallback, this, std::placeholders::_1));

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  
  }


  void GreenDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
  {
    // Get the transform from odom to gimbal
    // try
    // {
    //   rclcpp::Time target_time = img_msg->header.stamp;
    //   auto odom_to_gimbal = tf2_buffer_->lookupTransform(
    //       odom_frame_, "camera_optical_frame", target_time, rclcpp::Duration::from_seconds(0.008));
    //   auto msg_q = odom_to_gimbal.transform.rotation;
    //   tf2::Quaternion tf_q;
    //   tf2::fromMsg(msg_q, tf_q);
    //   tf2::Matrix3x3 tf2_matrix = tf2::Matrix3x3(tf_q);
    //   imu_to_camera_ << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1], tf2_matrix.getRow(0)[2],
    //       tf2_matrix.getRow(1)[0], tf2_matrix.getRow(1)[1], tf2_matrix.getRow(1)[2],
    //       tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1], tf2_matrix.getRow(2)[2];
    // }
    // catch (...)
    // {
    //   RCLCPP_INFO(rclcpp::get_logger("green_solver"),"Something Wrong when lookUpTransform");
    //   return;
    // }
    
    auto greens = detectGreen(img_msg);

    green_msg_.header = img_msg->header;

    if (pnp_solver_ != nullptr)
    {
      if(!greens.empty()){
        auto green_ = greens[0];
        cv::Mat rvec(3, 1, CV_64F), tvec(3, 1, CV_64F); // pnp return
        cv::Mat rotation_matrix(3, 3, CV_64F);

        // Use PnP to get the initial pose information
        if (pnp_solver_->solvePnP(green_.landmarks(), rvec, tvec, "green"))
        {
          cv::Rodrigues(rvec, rotation_matrix);

          // Fill pose
          green_msg_.pose.position.x = tvec.at<double>(0);
          green_msg_.pose.position.y = tvec.at<double>(1);
          green_msg_.pose.position.z = tvec.at<double>(2);

          green_msg_.pose_pixel.position.x = green_.center.x;
          green_msg_.pose_pixel.position.y = green_.center.y;

          // rotation matrix to quaternion
          tf2::Matrix3x3 tf2_rotation_matrix(rotation_matrix.at<double>(0, 0),
                                             rotation_matrix.at<double>(0, 1),
                                             rotation_matrix.at<double>(0, 2),
                                             rotation_matrix.at<double>(1, 0),
                                             rotation_matrix.at<double>(1, 1),
                                             rotation_matrix.at<double>(1, 2),
                                             rotation_matrix.at<double>(2, 0),
                                             rotation_matrix.at<double>(2, 1),
                                             rotation_matrix.at<double>(2, 2));
          tf2::Quaternion tf2_quaternion;
          tf2_rotation_matrix.getRotation(tf2_quaternion);
          green_msg_.pose.orientation.x = tf2_quaternion.x();
          green_msg_.pose.orientation.y = tf2_quaternion.y();
          green_msg_.pose.orientation.z = tf2_quaternion.z();
          green_msg_.pose.orientation.w = tf2_quaternion.w();

        }
      }
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("green_detector"),"PnP Failed!");
    }

    // Publishing detected armors
    green_pub_->publish(green_msg_);
  }

  std::vector<Green> GreenDetectorNode::detectGreen(
    const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
  {
    // Convert ROS img to cv::Mat
    auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

    auto greens_ = detector_->detect(img);

    auto final_time = this->now();
    auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;

    if (debug_)
    {
      binary_img_pub_.publish(
          cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());
    }

    detector_->drawResults(img);

    // Draw camera center
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
    // Draw latency
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(
        img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());

    return greens_;
  }

  rcl_interfaces::msg::SetParametersResult GreenDetectorNode::onSetParameters(
    std::vector<rclcpp::Parameter> parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters)
    {
      if (param.get_name() == "binary_thres")
      {
        detector_->binary_thres = param.as_int();
      }
      if (param.get_name() == "color_diff_thres")
      {
        detector_->color_diff_thres = param.as_int();
      }
    }

    return result;
  }

  std::unique_ptr<Detector> GreenDetectorNode::initDetector()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 255;

    int binary_thres = declare_parameter("binary_thres", 160, param_desc);

    int color_diff_thres = declare_parameter("color_diff_thres", 20, param_desc);

    auto detector = std::make_unique<Detector>(binary_thres, color_diff_thres);

    // Set dynamic parameter callback
    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&GreenDetectorNode::onSetParameters, this, std::placeholders::_1));

    return detector;
  }

  void GreenDetectorNode::createDebugPublishers() noexcept
  {
    // this->declare_parameter("armor_detector.result_img.jpeg_quality", 50);
    // this->declare_parameter("armor_detector.binary_img.jpeg_quality", 50);
    binary_img_pub_ = image_transport::create_publisher(this, "green_detector/binary_img");
    result_img_pub_ = image_transport::create_publisher(this, "green_detector/result_img");
  }

  void GreenDetectorNode::destroyDebugPublishers() noexcept
  {
    binary_img_pub_.shutdown();
    result_img_pub_.shutdown();
  }


} //namespace fyt::auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::GreenDetectorNode)