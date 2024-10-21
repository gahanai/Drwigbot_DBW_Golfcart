/**
 * @ Author: Pallab Maji
 * @ Create Time: 2024-10-09 16:38:13
 * @ Modified time: 2024-10-09 18:39:34
 * @ Description: This file contains the class declaration of EBA class.
 */

#pragma once

// Radar Node message headers
#include "radar_node/msg/array_tracked_obj.hpp"
#include "radar_node/msg/tracked_obj.hpp"

// Adas Interface message headers
#include "adas_interfaces/brake_command.hpp"
#include "adas_interfaces/dio_command.hpp"
#include "adas_interfaces/msg/brake_command.hpp"
#include "adas_interfaces/msg/steering_command.hpp"
#include "adas_interfaces/msg/system_command.hpp"
#include "adas_interfaces/msg/throttle_command.hpp"
#include "adas_interfaces/steering_command.hpp"
#include "adas_interfaces/system_command.hpp"
#include "adas_interfaces/throttle_command.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Std Libs
#include <map>
#include <stdint.h>
#include <vector>

class DrwigEBA : public rclcpp::Node
{
  public:
    DrwigEBA();
    void execute(radar_node::msg::ArrayTrackedObj::SharedPtr msg);
    void reset_dbw_system();
    ~DrwigEBA();

  private:
    // Constants
    float kMaxThrottle = 0.7f;
    float kMaxBrake = 1.0f;

    bool m_brake_status = false;

    rclcpp::Subscription<radar_node::msg::ArrayTrackedObj>::SharedPtr m_tracked_sub;
    rclcpp::Publisher<adas_interfaces::msg::BrakeCommand>::SharedPtr m_brake_pub;
    rclcpp::Publisher<adas_interfaces::msg::ThrottleCommand>::SharedPtr m_throttle_pub;
    rclcpp::Publisher<adas_interfaces::msg::SystemCommand>::SharedPtr m_dbw_system_pub;

    // Initialize the messages
    adas_interfaces::msg::BrakeCommand m_brake_cmd_msg;
    adas_interfaces::msg::ThrottleCommand m_throttle_cmd_msg;
    adas_interfaces::msg::SystemCommand m_system_cmd_msg;

}; // End of DrwigEBA class
