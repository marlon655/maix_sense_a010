#ifndef NAV_HUB_ACTIONS_MOVE_HPP
#define NAV_HUB_ACTIONS_MOVE_HPP

#include <iostream>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class RobotActions : public rclcpp::Node {
public:
    RobotActions();
    void rotate_on_axis();

private:
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    double normalize_angle(double angle);

    double current_yaw;
    bool yaw_initialized;
    geometry_msgs::msg::Twist velocity;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cor_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
};

#endif

