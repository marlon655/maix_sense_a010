#include "nav_hub/actions_move.hpp"

RobotActions::RobotActions() : Node("robot_actions") {
    current_yaw = 0.0;
    yaw_initialized = false;

    velocity = geometry_msgs::msg::Twist();

    pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/map_pose", 10, std::bind(&RobotActions::pose_callback, this, std::placeholders::_1));
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    cor_pub = this->create_publisher<std_msgs::msg::String>("cor", 10);
}

void RobotActions::rotate_on_axis() {
    // Aguarda leitura inicial da orientação
    RCLCPP_INFO(this->get_logger(), "Aguardando orientação inicial...");
    while (!yaw_initialized && rclcpp::ok()) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        rclcpp::spin_some(this->get_node_base_interface());
    }
    
    // Publica cor
    // auto cor_msg = std_msgs::msg::String();
    // cor_msg.data = "Color";
    // cor_pub->publish(cor_msg);
    
    // Salva orientação inicial
    double orientation = current_yaw;
    RCLCPP_INFO(this->get_logger(), "Orientação inicial salva: %.2f graus", orientation * 180.0 / M_PI);

    velocity.linear.x = 0.0;
    velocity.angular.z = -0.5;

    double duration = 3.0;
    
    // Gira por 3 segundos primeiro
    auto tempo_inicio = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        auto tempo_atual = std::chrono::steady_clock::now();
        auto tempo_decorrido = std::chrono::duration_cast<std::chrono::duration<double>>(tempo_atual - tempo_inicio).count();
        
        if (tempo_decorrido >= duration) {
            break;
        }
        
        cmd_vel_pub->publish(velocity);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
        rclcpp::spin_some(this->get_node_base_interface());
    }
    
    // Continue girando até completar uma volta completa
    while (rclcpp::ok()) {
        cmd_vel_pub->publish(velocity);
        rclcpp::spin_some(this->get_node_base_interface());

        // Diferença angular entre atual e inicial
        double angle_diff = normalize_angle(current_yaw - orientation);

        // Se completou 360 graus (ou quase)
        if (std::abs(angle_diff) < 0.05 && std::abs(angle_diff) > 0) {
            RCLCPP_INFO(this->get_logger(), "Orientação inicial alcançada novamente. Parando...");
            break;
        }
        
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    // Para o robô
    velocity.angular.z = 0.0;
    cmd_vel_pub->publish(velocity);
}

void RobotActions::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    auto orientation = msg->pose.pose.orientation;

    tf2::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw);
}

double RobotActions::normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}



