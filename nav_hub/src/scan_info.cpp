#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>

class ScanCounterNode : public rclcpp::Node
{
public:
    ScanCounterNode() : Node("scan_counter_node")
    {
        // Criar o subscriber para o tópico /scan
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&ScanCounterNode::scan_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "ScanCounterNode iniciado. Aguardando mensagens do tópico /scan...");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Obter a quantidade total de ranges
        size_t total_ranges = msg->ranges.size();
        
        // Contar ranges válidos (não infinitos e não NaN)
        size_t valid_ranges = 0;
        size_t invalid_ranges = 0;
        
        for (const auto& range : msg->ranges) {
            if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max) {
                valid_ranges++;
            } else {
                invalid_ranges++;
            }
        }
        
        // Informações do scan
        RCLCPP_INFO(this->get_logger(), 
                    "=== LASER SCAN INFO ===\n"
                    "Total de ranges: %zu\n"
                    "Ranges válidos: %zu\n"
                    "Ranges inválidos: %zu\n"
                    "Ângulo mínimo: %.2f rad (%.2f°)\n"
                    "Ângulo máximo: %.2f rad (%.2f°)\n"
                    "Incremento angular: %.4f rad (%.4f°)\n"
                    "Distância mínima: %.2f m\n"
                    "Distância máxima: %.2f m\n"
                    "Frame ID: %s\n"
                    "=====================",
                    total_ranges,
                    valid_ranges,
                    invalid_ranges,
                    msg->angle_min, msg->angle_min * 180.0 / M_PI,
                    msg->angle_max, msg->angle_max * 180.0 / M_PI,
                    msg->angle_increment, msg->angle_increment * 180.0 / M_PI,
                    msg->range_min,
                    msg->range_max,
                    msg->header.frame_id.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ScanCounterNode>();
    
    RCLCPP_INFO(node->get_logger(), "Executando ScanCounterNode...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}