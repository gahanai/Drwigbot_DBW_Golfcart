/**
 * @ Author: Pallab Maji
 * @ Create Time: 2024-10-09 16:30:47
 * @ Modified time: 2024-10-09 18:39:28
 * @ Description: This file contains the implementation of the class Drwig Emergency Brake Assist (EBA) class.
 *        The class is responsible for implementing the control logic from the Radar Node and operate the Throttle and
 *        Brake of the vehicle. Intialize the System Command to enable DBW, Brake and Throttle control.
 */

#include "drwig_adas/drwig_eba.hpp"

DrwigEBA::DrwigEBA() : Node("DrwigEBA")
{
    // Create the subscription to the tracked object data
    m_tracked_sub = this->create_subscription<radar_node::msg::ArrayTrackedObj>(
        "tracked_obj", 10, std::bind(&DrwigEBA::execute, this, std::placeholders::_1));

    // Create the publisher for the DBW System, Throttle & Brake Command
    m_brake_pub = this->create_publisher<adas_interfaces::msg::BrakeCommand>("cmd_brake", 10);
    m_throttle_pub = this->create_publisher<adas_interfaces::msg::ThrottleCommand>("cmd_throttle", 10);
    m_dbw_system_pub = this->create_publisher<adas_interfaces::msg::SystemCommand>("cmd_dbw_system", 10);

    // Initialize & Enable the system command message
    this->m_system_cmd_msg.header.frame_id = "DrwigEBA";
    this->m_system_cmd_msg.enable_dbw = true;
    this->m_system_cmd_msg.enable_brake = true;
    this->m_system_cmd_msg.enable_throttle = true;
    this->m_system_cmd_msg.enable_dio = true;
    this->m_system_cmd_msg.enable_logs = true;

    // Initialize the Brake Command message
    this->m_brake_cmd_msg.header.frame_id = "DrwigEBA";
    this->m_brake_cmd_msg.position = 0.0f;
    this->m_brake_cmd_msg.intensity = 100.0f;
    this->m_brake_cmd_msg.enable = true;

    // Initialize the Throttle Command message
    this->m_throttle_cmd_msg.header.frame_id = "DrwigEBA";
    this->m_throttle_cmd_msg.position = 0.0f;
    this->m_throttle_cmd_msg.intensity = 100.0f;
    this->m_throttle_cmd_msg.enable = true;

    // Log Successful Initialization using RCLCPP_INFO
    RCLCPP_INFO(this->get_logger(), "DrwigEBA Node Initialized Successfully");
}

void DrwigEBA::reset_dbw_system(void)
{
    // Reset the Drive-by-Wire system
    this->m_system_cmd_msg.enable_dbw = false;
    this->m_system_cmd_msg.enable_brake = false;
    this->m_system_cmd_msg.enable_throttle = false;
    this->m_system_cmd_msg.enable_dio = false;
    this->m_system_cmd_msg.enable_logs = false;

    this->m_brake_cmd_msg.enable = false;

    this->m_throttle_cmd_msg.enable = false;

    m_dbw_system_pub->publish(m_system_cmd_msg);
    m_brake_pub->publish(m_brake_cmd_msg);
    m_throttle_pub->publish(m_throttle_cmd_msg);
}

DrwigEBA::~DrwigEBA()
{

    // Shutdown the ROS2 node
    rclcpp::shutdown();

    // Log Successful shutdown
    RCLCPP_INFO(this->get_logger(), "DrwigEBA Node Shutdown Successfully");
}

void DrwigEBA::execute(const radar_node::msg::ArrayTrackedObj::SharedPtr msg)
{
    //  Read the tracked object data from the radar node and check if the brake status is on or off
    //  If the brake status is on, then send the brake command to the vehicle
    //  If the brake status is off, then send the throttle command to the vehicle

    radar_node::msg::ArrayTrackedObj array_tracked_obj = *msg;

    this->m_brake_status = msg->brake_status;

    this->m_throttle_cmd_msg.header.stamp = this->now();
    this->m_brake_cmd_msg.header.stamp = this->now();

    if (this->m_brake_status)
    {
        // Send the brake command to the vehicle
        m_throttle_cmd_msg.position = 0.0f;
        m_brake_cmd_msg.position = this->kMaxBrake;
    }
    else
    {
        // Send the throttle command to the vehicle
        m_brake_cmd_msg.position = 0.0f;
        m_throttle_cmd_msg.position = this->kMaxThrottle;
    }
    m_dbw_system_pub->publish(m_system_cmd_msg);
    m_brake_pub->publish(m_brake_cmd_msg);
    m_throttle_pub->publish(m_throttle_cmd_msg);

    // Log the brake and throttle command and the brake status
    RCLCPP_INFO(this->get_logger(), "Brake Command: %f, Throttle Command: %f, Brake Status: %d",
                m_brake_cmd_msg.position, m_throttle_cmd_msg.position, this->m_brake_status);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrwigEBA>());
    rclcpp::shutdown();
    return 0;
}