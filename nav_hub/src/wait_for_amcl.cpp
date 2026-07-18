#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include <chrono>
#include <memory>
#include <cstdlib>

using namespace std::chrono_literals;

class AmclWatchdog : public rclcpp::Node
{
public:
    AmclWatchdog() : Node("amcl_watchdog")
    {
        // Cliente para verificar se o AMCL está vivo
        get_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("amcl/get_state");
        
        // Parâmetros configuráveis
        this->declare_parameter("check_interval_seconds", 5.0);
        this->declare_parameter("max_retry_attempts", 5);
        this->declare_parameter("service_wait_timeout", 3.0);
        this->declare_parameter("restart_command", std::string("ros2 launch nav_hub launch_amcl.launch.py"));
        
        check_interval_ = this->get_parameter("check_interval_seconds").as_double();
        max_retry_attempts_ = this->get_parameter("max_retry_attempts").as_int();
        service_wait_timeout_ = this->get_parameter("service_wait_timeout").as_double();
        restart_command_ = this->get_parameter("restart_command").as_string();
        
        // Timer para verificar periodicamente se o AMCL está vivo
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(check_interval_), 
            std::bind(&AmclWatchdog::check_amcl_alive, this));
        
        RCLCPP_INFO(this->get_logger(), "AMCL Watchdog iniciado. Monitorando AMCL...");
        RCLCPP_INFO(this->get_logger(), "Intervalo de verificação: %.1f segundos", check_interval_);
        RCLCPP_INFO(this->get_logger(), "Máximo de tentativas: %d", max_retry_attempts_);
        
        if (!restart_command_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Comando de restart: %s", restart_command_.c_str());
        }
    }

private:
    void check_amcl_alive()
    {
        // Simplesmente verifica se o serviço está disponível
        auto service_timeout = std::chrono::duration<double>(service_wait_timeout_);
        
        if (!get_state_client_->wait_for_service(service_timeout)) {
            consecutive_failures_++;
            RCLCPP_WARN(this->get_logger(), 
                       "AMCL não responde (falhas consecutivas: %d)", 
                       consecutive_failures_);
            
            // Se falhou muitas vezes seguidas, assume que crashou
            if (consecutive_failures_ >= 3) {
                handle_amcl_crash();
            }
        } else {
            // Reset contador se está respondendo
            if (consecutive_failures_ > 0) {
                RCLCPP_INFO(this->get_logger(), "AMCL voltou a responder - OK");
                consecutive_failures_ = 0;
                crash_retry_count_ = 0; // Reset também o contador de crashes
            }
        }
    }
    
    void handle_amcl_crash()
    {
        if (crash_retry_count_ >= max_retry_attempts_) {
            RCLCPP_ERROR(this->get_logger(), 
                        "AMCL crashou %d vezes. Máximo de tentativas atingido. Parando tentativas.", 
                        crash_retry_count_);
            return;
        }
        
        crash_retry_count_++;
        RCLCPP_ERROR(this->get_logger(), 
                    "AMCL crashou! Tentativa de restart %d/%d", 
                    crash_retry_count_, max_retry_attempts_);
        
        if (!restart_command_.empty()) {
            restart_amcl_process();
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                        "Nenhum comando de restart configurado!");
        }
        
        // Reset contador de falhas consecutivas
        consecutive_failures_ = 0;
        
        // Agenda verificação para ver se reiniciou
        restart_check_timer_ = this->create_wall_timer(
            10s, [this]() {
                RCLCPP_INFO(this->get_logger(), "Verificando se AMCL reiniciou...");
                check_amcl_alive();
                restart_check_timer_->cancel();
            });
    }
    
    void restart_amcl_process()
    {
        RCLCPP_INFO(this->get_logger(), "Executando: %s", restart_command_.c_str());
        
        // Executa o comando em background
        std::string full_command = restart_command_ + " &";
        int result = std::system(full_command.c_str());
        
        if (result == 0) {
            RCLCPP_INFO(this->get_logger(), "Comando executado com sucesso");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Falha ao executar comando (código: %d)", result);
        }
    }
    
private:
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr restart_check_timer_;
    
    double check_interval_;
    int max_retry_attempts_;
    double service_wait_timeout_;
    std::string restart_command_;
    
    int crash_retry_count_ = 0;
    int consecutive_failures_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<AmclWatchdog>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("amcl_watchdog"), "Exceção: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}