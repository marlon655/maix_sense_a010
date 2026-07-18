#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <cmath>
#include <thread>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav_hub/json.hpp"
#include "nav_hub/costmap_cleaner.hpp"
#include "nav_hub/log_manager.hpp"

using ordered_json = nlohmann::ordered_json;
using namespace std::chrono_literals;

bool publish_once_flag = false;

class NavStack : public rclcpp::Node {

public:
    NavStack() : Node("main_route_graph")
    {
        costmap_cleaner = std::make_shared<CostmapCleaner>();
        log_manager = std::make_shared<LogManager>(); 
        
        //QoS definitions
        auto color_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        auto sound_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        auto default_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        auto battery_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        auto destination_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        // Subscribers
        destination_sub = this->create_subscription<std_msgs::msg::Int32>("destination", destination_qos, std::bind(&NavStack::destination_callback, this, std::placeholders::_1));
        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&NavStack::velocity_callback, this, std::placeholders::_1));
        battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>("battery_state", battery_qos, std::bind(&NavStack::battery_callback, this, std::placeholders::_1));
        nav_status_sub = this->create_subscription<action_msgs::msg::GoalStatusArray>("navigate_to_pose/_action/status", 10, std::bind(&NavStack::nav_status_callback, this, std::placeholders::_1));

        // Publishers
        sound_pub = this->create_publisher<std_msgs::msg::String>("som", sound_qos);
        color_pub = this->create_publisher<std_msgs::msg::String>("cor", color_qos);
        position_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
        reached_pub = this->create_publisher<std_msgs::msg::Bool>("has_reached", default_qos);
        navigating_pub = this->create_publisher<std_msgs::msg::Bool>("navegando", 10);
        // clicked_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/clicked_point", 50);
        goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        particle_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", 10);
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        timer_ = this->create_wall_timer(500ms, std::bind(&NavStack::navigate, this));

        RCLCPP_INFO(this->get_logger(), "NavStack initiated!");

    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_pub;


private:
    void load_json_data(const std::string& file_path) {
        try {
            std::ifstream file(file_path);
            if(!file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Não foi possível abrir o arquivo JSON: %s", file_path.c_str());
                return;
            }

            file >> route_data;
            file.close();

            RCLCPP_INFO(this->get_logger(), "Arquivo JSON carregado com sucesso.");
        }
        catch(const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao carregar arquivo JSON: %s", e.what());
            return;
        }
    }

    void clear_around_costmaps(int meters = 5) { // por default vai limpar uma area de 5x5 ao redor do robo, isso é totalmentee customizavel
        costmap_cleaner->clearCostmapsAroundRobot(meters);
    }

    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        velocity = *msg;
    }

    void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        voltage_battery = msg->voltage;
    }

    void nav_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        if (!msg->status_list.empty()) {
            auto status = msg->status_list.back().status;
            if (status == 4) {  // STATUS_SUCCEEDED
                flag_reached = true;
                em_movimento = false;
                RCLCPP_INFO(this->get_logger(), "Objetivo alcançado!");
            } else if (status == 6) {  // STATUS_ABORTED
                RCLCPP_WARN(this->get_logger(), "Navegação abortada!");
                log_manager->gera_log("Navigation aborted!", LogManager::Error);
                publish_color("Apagado");
            }
        }
    }

    void publish_sound(const std::string& sound) {
        sound_pub->publish(std_msgs::msg::String().set__data(sound));
    }

    void publish_color(const std::string& color) {
        color_pub->publish(std_msgs::msg::String().set__data(color));
    }

    void destination_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received destination %d", msg->data);
        std::string destino = std::to_string(msg->data);        
        // Se for o início, define pose inicial
        if(beginning) {
            load_json_data(caminho_rota);
            beginning = false;
            
            auto position = geometry_msgs::msg::PoseWithCovarianceStamped();
            position.header.stamp = this->now();
            position.header.frame_id = "map";
            position.pose.pose.position.x = route_data["initial"]["x"];
            position.pose.pose.position.y = route_data["initial"]["y"];
            position.pose.pose.position.z = 0.0;
            position.pose.pose.orientation.z = route_data["initial"]["z"];
            position.pose.pose.orientation.w = route_data["initial"]["w"];
            
            std::vector<double> covariance = route_data["initial"]["covariance"];
            for(size_t i = 0; i < covariance.size() && i < 36; ++i) {
                position.pose.covariance[i] = covariance[i];
            }
            
            position_pub->publish(position);
            RCLCPP_INFO(this->get_logger(), "Posição inicial configurada no ponto %s", destino.c_str());
            log_manager->gera_log("Initial position set to point " + destino, LogManager::Info);
        }

        // clear_costmaps();

        std::this_thread::sleep_for(2s);

        int sequence = std::stoi(destino);
        auto goal = geometry_msgs::msg::PoseStamped();
        goal.header.stamp = this->now();
        goal.header.frame_id = "map";

        if(sequence == 1) {
            goal.pose.position.x = route_data["initial"]["x"];
            goal.pose.position.y = route_data["initial"]["y"];
            goal.pose.position.z = 0.0;
            goal.pose.orientation.z = route_data["initial"]["z"];
            goal.pose.orientation.w = route_data["initial"]["w"];
        } else {
            for(auto& [key, val] : route_data.items()) {
                if(key != "initial") {
                    try {
                        if(val.contains("sequence") && val["sequence"] == sequence) {
                            goal.pose.position.x = val["x"];
                            goal.pose.position.y = val["y"];
                            goal.pose.position.z = 0.0;
                            goal.pose.orientation.z = val["z"];
                            goal.pose.orientation.w = val["w"];
                            // sequence++;
                            break;
                        }
                    } catch(...) {
                        RCLCPP_WARN(this->get_logger(), "Error processing route point: %s", key.c_str());
                    }
                }
                if(key == "ending") {
                    break;
                }
                
            }
        }
        
        goal_pub->publish(goal);
        navegando = true;
        navigating_pub->publish(std_msgs::msg::Bool().set__data(true));
        reached_pub->publish(std_msgs::msg::Bool().set__data(false));
        publish_color("Ciano");
        publish_sound("Andando");
        RCLCPP_INFO(this->get_logger(), "Robô em movimento.");
        log_manager->gera_log("Robot moving to destination " + destino + " ",  LogManager::Info);
    }

    void navigate() {
        if(!iniciou) {
            iniciou = true;
            navigating_pub->publish(std_msgs::msg::Bool().set__data(false));
            
            // clear_costmaps();
            reached_pub->publish(std_msgs::msg::Bool().set__data(true));
            publish_color("Roxo");
            publish_sound("Pronto");
            RCLCPP_INFO(this->get_logger(), "Navegação route graph pronta.");
            log_manager->gera_log("Route graph navigation ready. Total routes: 1" , LogManager::Info);
        }
        // Determina se o robô chegou ou não
        else if(flag_reached) {
            flag_reached = false;
            navegando = false;
            navigating_pub->publish(std_msgs::msg::Bool().set__data(false));
            RCLCPP_INFO(this->get_logger(), "Route graph detectou objetivo alcançado.");
            RCLCPP_INFO(this->get_logger(), "Tensão na bateria: %.2f", voltage_battery);
            log_manager->gera_log("Goal reached. Battery voltage: " + std::to_string(voltage_battery), LogManager::Info);
            
            reached_pub->publish(std_msgs::msg::Bool().set__data(true));
            publish_sound("Pronto");
            publish_color("Verde");

            std::this_thread::sleep_for(1s);

            // clear_costmaps();
        }
        // Quando uma rota é enviada para o sistema de navegação
        else if(navegando) {
            bool robot_stopped = (std::abs(velocity.linear.x) <= 0.0062 &&
                                 std::abs(velocity.angular.z) <= 0.037);

            if (robot_stopped) {
                if (!em_movimento) {
                    em_movimento = true;
                    last_movement_time = std::chrono::steady_clock::now();
                    last_costmap_clear_time = std::chrono::steady_clock::now();
                    publish_sound("Parado");
                    publish_color("Amarelo");
                    RCLCPP_INFO(this->get_logger(), "Robô parou devido a obstáculo.");
                    log_manager->gera_log("Robot stopped due to obstacle.", LogManager::Warn);
                } else {
                    auto current_time = std::chrono::steady_clock::now();
                    auto time_stopped = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_movement_time).count();
                    auto time_since_last_clear = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_costmap_clear_time).count();

                    if (time_stopped >= max_elapsed_time_to_clear_costmaps && time_since_last_clear >= max_elapsed_time_to_clear_costmaps) {
                        last_costmap_clear_time = current_time;
                        clear_around_costmaps();
                        RCLCPP_INFO(this->get_logger(), "Robô parado por %ld segundos. Limpando costmaps.", time_stopped);
                        log_manager->gera_log("Robot stopped for " + std::to_string(time_stopped) + " seconds. Attempting to clear costmaps.", LogManager::Warn);
                    }
                }
            } else {
                if (em_movimento) {
                    em_movimento = false;
                    publish_sound("Andando");
                    publish_color("Ciano");
                    RCLCPP_INFO(this->get_logger(), "Robô em movimento.");
                    log_manager->gera_log("Robot resumed movement.", LogManager::Info);
                }
            }
        }
    }

    //Variaveis private:
    // std::string caminho_rota = "/home/ubuntu/ros2_ws/src/nav_hub/config/ponto-ponto-demo/teste_fora/";
    std::string caminho_rota = "/home/ubuntu/ros2_ws/src/nav_hub/route/route_graph/goals_oregon.json";
    // std::string caminho_rota = "/home/ubuntu/ros2_ws/src/nav_hub/route/route_graph/goals_aceleradora_teste_oregon.json";
    // std::string caminho_rota = "/home/ubuntu/ros2_ws/src/nav_hub/route/route_graph/goals_aceleradora_teste_oregon_2.json";
    // std::string caminho_rota = "/home/ubuntu/ros2_ws/src/nav_hub/route/route_graph/goals_aceleradora_teste_oregon_3.json";
    
    ordered_json route_data;
    bool beginning = true;
    std::vector<double> current_pose = {0.0, 0.0};
    double current_yaw = 0.0;
    bool flag_reached = false;
    bool iniciou = false;
    bool navegando = false;
    bool em_movimento = false;
    double voltage_battery = 0.0;
    int seq_counter = 0;
    int max_elapsed_time_to_clear_costmaps = 6;
    std::chrono::steady_clock::time_point last_movement_time;
    std::chrono::steady_clock::time_point last_costmap_clear_time;

    std::shared_ptr<CostmapCleaner> costmap_cleaner;
    std::shared_ptr<LogManager> log_manager;
    geometry_msgs::msg::Twist velocity;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr destination_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr position_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr navigating_pub;
    // rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr clicked_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sound_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavStack>();

    RCLCPP_INFO(node->get_logger(), "Iniciando main_route_graph.cpp...");
    rclcpp::spin(node);

    // Apaga a fita LED
    if(!publish_once_flag) {
        auto cor_msg = std_msgs::msg::String();
        cor_msg.data = "Apagado";
        node->color_pub->publish(cor_msg);
    }

    rclcpp::shutdown();
    return 0;
}