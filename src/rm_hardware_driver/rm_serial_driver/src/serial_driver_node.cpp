// Created by Chengfu Zou on 2023.7.1
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_serial_driver/serial_driver_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
// std
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
// ros2
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
// project
#include "rm_serial_driver/uart_transporter.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace fyt::serial_driver
{
  SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions &options)
      : Node("serial_driver", options)
  {
    FYT_REGISTER_LOGGER("serial_driver", "~/fyt2024-log", INFO);

    // Task thread
    listen_thread_ = std::make_unique<std::thread>(&SerialDriverNode::listenLoop, this);
    FYT_INFO("serial_driver", "Initializing SerialDriverNode!");
  }

  void SerialDriverNode::init()
  {
    // Init
    target_frame_ = this->declare_parameter("target_frame", "odom");
    std::string port_name = this->declare_parameter("port_name", "/dev/ttyUSB0");
    std::string protocol_type = this->declare_parameter("protocol", "infantry");
    bool enable_data_print = this->declare_parameter("enable_data_print", false);
    bool enable_send_data_print = this->declare_parameter("enable_send_data_print", false);
    // Create Protocol
    protocol_ = ProtocolFactory::createProtocol(protocol_type, port_name, enable_data_print, enable_send_data_print);
    if (protocol_ == nullptr)
    {
      FYT_FATAL("serial_driver", "Failed to create protocol with type: {}", protocol_type);
      rclcpp::shutdown();
      return;
    }
    FYT_INFO(
        "serial_driver", "Protocol has been created with type: {}, port: {}", protocol_type, port_name);

    // Subscriptions
    subscriptions_ = protocol_->getSubscriptions(this->shared_from_this());
    for (auto sub : subscriptions_)
    {
      FYT_INFO("serial_driver", "Subscribe to topic: {}", sub->get_topic_name());
    }
    // Publisher
    serial_receive_data_pub_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>(
        "serial/receive", rclcpp::SensorDataQoS());

    // TF broadcaster
    timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Param client
    for (auto client : protocol_->getClients(this->shared_from_this()))
    {
      std::string name = client->get_service_name();
      set_mode_clients_.emplace(name, client);
      FYT_INFO("serial_driver", "Create client for service: {}", name);
    }

    for (auto client : protocol_->getParamClients(this->shared_from_this()))
    {
      set_params_clients_.emplace_back(client);
      FYT_INFO("serial_driver", "Create param client for node: {}", this->get_name());
    }

    // Heartbeat
    heartbeat_ = HeartBeatPublisher::create(this);

    FYT_INFO("serial_driver", "SerialDriverNode has been initialized!");
  }

  SerialDriverNode::~SerialDriverNode()
  {
    FYT_INFO("serial_driver", "Destroy SerialDriverNode!");
    rclcpp::shutdown();
    if (listen_thread_ != nullptr)
    {
      listen_thread_->join();
    }
  }

  void SerialDriverNode::listenLoop()
  {
    if (protocol_ == nullptr)
    {
      // Lazy init because shared_from_this() is not available in constructor
      init();
    }

    rm_interfaces::msg::SerialReceiveData receive_data;
    while (rclcpp::ok())
    {
      if (protocol_->receive(receive_data))
      {
        receive_data.header.stamp = this->now() - rclcpp::Duration::from_seconds(timestamp_offset_);
        receive_data.header.frame_id = target_frame_;
        serial_receive_data_pub_->publish(receive_data);

        // for (auto &[service_name, client] : set_mode_clients_)
        // {
        //   if (client.mode.load() != receive_data.mode && !client.on_waiting.load())
        //   {
        //     setMode(client, receive_data.mode);
        //   }
        // }
        // if (!initial_set_param_ || receive_data.detect_color != previous_receive_color_)
        //{
        // for (auto &client : set_params_clients_)
        // {
        //   if (receive_data.detect_color < 2 && client.enemy_color != receive_data.detect_color && !client.on_waiting)
        //   {
        //     setParam(client, rclcpp::Parameter("detect_color", receive_data.detect_color));
        //   }
        // }
        //}

      }
      else
      {
        auto error_message = protocol_->getErrorMessage();
        error_message = error_message.empty() ? "unknown" : error_message;
        // FYT_WARN("serial_driver", "Failed to reveive packet! error message :{}", error_message);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
  }

  void SerialDriverNode::setMode(SetModeClient &client, const uint8_t mode)
  {
    using namespace std::chrono_literals;

    std::string service_name = client.ptr->get_service_name();
    // Wait for service
    while (!client.ptr->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        FYT_ERROR(
            "serial_driver", "Interrupted while waiting for the service {}. Exiting.", service_name);
        return;
      }
      FYT_INFO("serial_driver", "Service {} not available, waiting again...", service_name);
    }
    if (!client.ptr->service_is_ready())
    {
      FYT_WARN("serial_driver", "Service: {} is not available!", service_name);
      return;
    }
    // Send request
    auto req = std::make_shared<rm_interfaces::srv::SetMode::Request>();
    req->mode = mode;

    client.on_waiting.store(true);
    auto result = client.ptr->async_send_request(
        req, [mode, &client](rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture result)
        {
      client.on_waiting.store(false);
      if (result.get()->success) {
        client.mode.store(mode);
      } });
  }

  void fyt::serial_driver::SerialDriverNode::setParam(SetParamsClient &client, const rclcpp::Parameter &param)
  {
    if (!client.ptr->service_is_ready())
    {
      RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
      return;
    }

    if (
        !client.set_param_future.valid() ||
        client.set_param_future.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
    {
      RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
      client.on_waiting = true;
      client.set_param_future = client.ptr->set_parameters(
          {param}, [this, param, &client](const ResultFuturePtr &results)
          {
           client.on_waiting=false;
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
       client.enemy_color=(static_cast<int>(param.as_int())); });
    }
  }

} // namespace fyt::serial_driver

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::serial_driver::SerialDriverNode)
