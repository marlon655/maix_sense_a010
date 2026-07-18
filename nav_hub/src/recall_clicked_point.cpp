#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

class GlobalPlanHandler : public rclcpp::Node {
public:
    GlobalPlanHandler() : Node("recall_clicked_point_node") {
        // Inicializa variáveis
        velocidade = geometry_msgs::msg::Twist();
        pontos.clear();
        last_index_rout = 0;
        flag_new_route = false;
        started = false;
        iniciou_rota = false;
        start_timer = true;
        elapsed_time = 0.0;
        wait_time = 5.0;  // 5 segundos
        reached = false;
        robot_x = 0.0;
        robot_y = 0.0;

        //Inicializa client
        cancel_goal_client = this->create_client<action_msgs::srv::CancelGoal>("/navigate_to_pose/_action/cancel_goal");
        auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();

        clear_costmap_client = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");

        // Inicializa publishers
        clicked_point_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/clicked_point", 50);
        cor_pub = this->create_publisher<std_msgs::msg::String>("/cor", 1);
        
        // Inicializa subscribers
        reached_sub = this->create_subscription<std_msgs::msg::Bool>(
            "has_reached", 1, std::bind(&GlobalPlanHandler::reached_callback, this, std::placeholders::_1));
        
        global_plan_sub = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 1, std::bind(&GlobalPlanHandler::new_global_plan_callback, this, std::placeholders::_1));
        
        new_clicked_point_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/new_clicked_point", 10, std::bind(&GlobalPlanHandler::new_clicked_point_callback, this, std::placeholders::_1));
        
        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 1, std::bind(&GlobalPlanHandler::velocidade_callback, this, std::placeholders::_1));
        
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/map_pose", 1, std::bind(&GlobalPlanHandler::pose_callback, this, std::placeholders::_1));
        
        
        process_timer = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&GlobalPlanHandler::process_route, this));
        
        RCLCPP_INFO(this->get_logger(), "GlobalPlanHandler node started");
    }

private:
    void reached_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        reached = msg->data;
        
        if (!started) {
            dist_epsilon = 0.01; // Pra pegar os parametros dinamicamente no ros2, tem q chamar um service get parameters, nao consegui entender como fazer isso com o waypoint global planner
            pontos_por_metro = 20.0; //Caso mude na config vai ter q mudar aq na mao memo
            dist_entre_pontos = 1.0 / pontos_por_metro;
            
            started = true;
        }
        
        if (reached) {
            flag_new_route = false;
            iniciou_rota = false;
            pontos.clear();
            start_timer = false;
        }
    }
    
    void new_global_plan_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!iniciou_rota) {
            rota_inicial = *msg;
            last_index_rout = 0;
            iniciou_rota = true;
        } else {
            flag_new_route = true;
        }
    }
    
    void new_clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        pontos.push_back(msg->point);
        clicked_point_pub->publish(*msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    void velocidade_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        velocidade = *msg;
    }
    
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        robot_x = msg->pose.pose.position.x;
        robot_y = msg->pose.pose.position.y;
    }
    
    void clear_costmaps() {
        if (!clear_costmap_client->wait_for_service(std::chrono::milliseconds(100))) {
            RCLCPP_WARN(this->get_logger(), "Clear costmap service not available");
            return;
        }
        
        auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
        auto future = clear_costmap_client->async_send_request(request);
        
        // Não bloquear - apenas enviar a requisição
        if (future.wait_for(std::chrono::milliseconds(1000)) == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "Costmaps cleared successfully");
        }
    }
    
    void publish_points(const std::vector<geometry_msgs::msg::Point>& pontos) {
        clear_costmaps();
        
        for (const auto& ponto : pontos) {
            auto point_msg = geometry_msgs::msg::PointStamped();
            point_msg.header.frame_id = "map";
            point_msg.header.stamp = this->now();
            point_msg.point = ponto;
            
            clicked_point_pub->publish(point_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    double distancia_euclidiana(const std::vector<double>& ponto_a, const std::vector<double>& ponto_b) {
        return std::sqrt(std::pow(ponto_a[0] - ponto_b[0], 2) + std::pow(ponto_a[1] - ponto_b[1], 2));
    }
    
    int position_at_route() {
        int indice = 0;
        double dist_minima_robot_rout = std::numeric_limits<double>::infinity();
        
        for (size_t i = last_index_rout; i < rota_inicial.poses.size(); ++i) {
            const auto& pose = rota_inicial.poses[i];
            double dist = distancia_euclidiana(
                {robot_x, robot_y}, 
                {pose.pose.position.x, pose.pose.position.y}
            );
            
            if (dist < dist_minima_robot_rout) {
                dist_minima_robot_rout = dist;
                indice = i;
            }
        }
        
        return indice;
    }
    
    void process_route() {
        if (flag_new_route) {
            last_index_rout = position_at_route();
            
            size_t comprimento_rota = rota_inicial.poses.size();
            std::vector<geometry_msgs::msg::Point> pontos_na_rota;
            
            // Verifica quais pontos estão na rota
            for (const auto& ponto : pontos) {
                size_t start_idx = (last_index_rout + static_cast<int>(pontos_por_metro) < static_cast<int>(comprimento_rota - pontos_por_metro)) 
                    ? last_index_rout + static_cast<int>(pontos_por_metro) 
                    : last_index_rout;
                
                for (size_t i = start_idx; i < rota_inicial.poses.size(); ++i) {
                    const auto& pose = rota_inicial.poses[i];
                    double dist = distancia_euclidiana(
                        {ponto.x, ponto.y}, 
                        {pose.pose.position.x, pose.pose.position.y}
                    );
                    
                    if (dist < dist_entre_pontos) {
                        pontos_na_rota.push_back(ponto);
                        break;
                    }
                }
            }
            
            // Adiciona pontos se necessário
            if (pontos_na_rota.size() == 0) {
                if (rota_inicial.poses.size() >= 2) {
                    pontos_na_rota.push_back(rota_inicial.poses[rota_inicial.poses.size()-2].pose.position);
                    pontos_na_rota.push_back(rota_inicial.poses.back().pose.position);
                }
            } else if (pontos_na_rota.size() == 1) {
                if (rota_inicial.poses.size() >= 2) {
                    pontos_na_rota.insert(pontos_na_rota.begin(), rota_inicial.poses[rota_inicial.poses.size()-2].pose.position);
                }
            }
            
            // Cancela objetivo atual
            auto cancel_msg = action_msgs::msg::GoalInfo();
            // cancel_pub->publish(cancel_msg);

            auto future = cancel_goal_client->async_send_request(request,
            [this](rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture future_response) {
                auto response = future_response.get();
                RCLCPP_INFO(this->get_logger(), "Cancelled %zu goals.", response->goals_canceling.size());
            });
            
            // Publica novos pontos
            publish_points(pontos_na_rota);
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            flag_new_route = false;
            
        } else if (iniciou_rota) {
            // Verifica se robô está parado
            if (std::abs(velocidade.linear.x) <= 0.0062 && std::abs(velocidade.angular.z) <= 0.037) {
                if (!start_timer) {
                    start_timer = true;
                    start_time = std::chrono::steady_clock::now();
                    elapsed_time = 0.0;
                }
                
                auto current_time = std::chrono::steady_clock::now();
                elapsed_time = std::chrono::duration<double>(current_time - start_time).count();
                
                if (elapsed_time >= wait_time) {
                    flag_new_route = true;
                    start_timer = false;
                }
            } else {
                if (start_timer) {
                    start_timer = false;
                }
            }
        }
    }
    
    // Variáveis de controle
    geometry_msgs::msg::Twist velocidade;
    std::vector<geometry_msgs::msg::Point> pontos;
    int last_index_rout;
    bool flag_new_route;
    bool started;
    bool iniciou_rota;
    nav_msgs::msg::Path rota_inicial;
    bool start_timer;
    std::chrono::steady_clock::time_point start_time;
    double elapsed_time;
    double wait_time;
    bool reached;
    
    // Parâmetros
    double dist_epsilon;
    double pontos_por_metro;
    double dist_entre_pontos;
    
    // Posição do robô
    double robot_x;
    double robot_y;

    //Clients
    rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_goal_client;
    std::shared_ptr<action_msgs::srv::CancelGoal::Request> request;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cor_pub;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr new_clicked_point_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
    
    // Timer para processamento
    rclcpp::TimerBase::SharedPtr process_timer;
    
    // Cliente para limpar costmaps
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_client;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalPlanHandler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}