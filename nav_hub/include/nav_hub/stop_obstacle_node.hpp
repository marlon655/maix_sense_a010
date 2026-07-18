#ifndef NAV_HUB__STOP_OBSTACLE_NODE_HPP_
#define NAV_HUB__STOP_OBSTACLE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <cmath>
#include <string>
#include <optional>

class StopObstacle : public rclcpp::Node
{
public:
    StopObstacle();

private:
    // --- Callbacks ---
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void global_plan_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void reached_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // --- Métodos Principais (espelhando o Python) ---
    void run();
    std::vector<std::pair<double, double>> get_front_obstacles();
    std::optional<std::pair<double, double>> lidar_to_global(double lidar_x, double lidar_y);
    bool comparePosition();
    double euclideanDistance(double x1, double y1, double x2, double y2);
    size_t positionAtRout();
    geometry_msgs::msg::Twist smooth_transition(geometry_msgs::msg::Twist vel_inicial, geometry_msgs::msg::Twist vel_final, double duration);

    // --- Timer ---
    rclcpp::TimerBase::SharedPtr timer_;
    
    // --- Publishers e Subscribers ---
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub_;

    // --- TF ---
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // --- Variáveis de Estado (espelhando o Python) ---
    geometry_msgs::msg::Twist velocity_;
    geometry_msgs::msg::Twist lastVel_;
    
    bool FlagIniciouRotina_ = false;
    bool started_ = false;
    nav_msgs::msg::Path::SharedPtr rotaInicial_;
    bool iniciouRota_ = false;
    bool flagIniciouScan_ = false;
    size_t LastIndexRout_ = 0;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    bool reached_ = false;

    // --- Dados do Scan ---
    std::vector<float> laser_ranges_;
    double angle_min_ = 0.0;
    double angle_increment_ = 0.0;
    std::string laser_frame_id_ = "";

    // --- Parâmetros de Configuração (espelhando o Python) ---
    double stopDistance_;
    double larguraDoRobo_;
    double dist_minima_;
    double passo_globalPath_;
    double anguloDireita_;
    double anguloEsquerda_;
    double dist_entre_pontos_ = 0.05; // Valor padrão, será calculado dinamicamente.
};

#endif // NAV_HUB__STOP_OBSTACLE_NODE_HPP_

