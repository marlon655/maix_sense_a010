#include <chrono>
#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <queue>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_hub/log_manager.hpp"
#include <algorithm>

using std::placeholders::_1;
using namespace std::chrono_literals;

// === Constantes Globais ===
const int MAX_QUEUE_SIZE = 4;
const std::string QUEUE_FILE_PATH = "/home/ubuntu/Desktop/tmp/queue.txt";
const int HOME = 1; // ponto de descanso do robo
const int DESCARTE = 2;
const int CENTRIFUGA = 3;
const int FORNO_UM = 4; // esse 4 é um button_id
const int FORNO_DOIS = 5;

// Função utilitária para obter a data atual.
std::string get_current_date() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    struct tm* timeinfo = localtime(&time_t_now);
    char buffer[11];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d", timeinfo);
    return std::string(buffer);
}

class BotoeiraNode : public rclcpp::Node
{
public:
    BotoeiraNode() : Node("botoeira_ponto_ponto_node") {
        log_manager = std::make_shared<LogManager>();
        
        // carregar fila se existir e for do mesmo dia
        load_queue_from_file();

        //QoS definitions
        auto color_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        auto sound_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        auto oled_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        auto battery_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        auto config_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        auto accepted_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
        auto destination_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

        // Publishers
        destination_pub = this->create_publisher<std_msgs::msg::Int32>("destination", destination_qos);
        cor_pub = this->create_publisher<std_msgs::msg::String>("/cor", color_qos);
        som_pub = this->create_publisher<std_msgs::msg::String>("/som", sound_qos);
        oled_pub = this->create_publisher<std_msgs::msg::String>("/oled", oled_qos);
        accepted_pub = this->create_publisher<std_msgs::msg::Bool>("request_accepted", accepted_qos);
        deploy_pub = this->create_publisher<std_msgs::msg::Bool>("abrir_engate", accepted_qos);

        // Subscribers
        reached_sub = this->create_subscription<std_msgs::msg::Bool>(
            "has_reached", config_qos, std::bind(&BotoeiraNode::reached_callback, this, _1));
        
        total_rotas_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/total_rotas", config_qos, std::bind(&BotoeiraNode::total_rotas_callback, this, _1));

        button_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/button", volatile_qos, std::bind(&BotoeiraNode::button_callback, this, _1));
        
        lora_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/lora_message", volatile_qos, std::bind(&BotoeiraNode::lora_callback, this, _1));

        battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery_state", battery_qos, std::bind(&BotoeiraNode::battery_callback, this, std::placeholders::_1));

        deployed_sub = this->create_subscription<std_msgs::msg::Bool>(
            "desengatado", config_qos, std::bind(&BotoeiraNode::deployed_callback, this, _1));

        hardware_off = this->create_publisher<std_msgs::msg::Bool>("desliga_hardware", config_qos);

        controller_param_client = std::make_shared<rclcpp::AsyncParametersClient>(
            this, "/controller_server");

        speed_limit_pub = this->create_publisher<nav2_msgs::msg::SpeedLimit>("speed_limit", rclcpp::QoS(1).reliable());
        
        controller_param_client->wait_for_service(1s);

        if (ENABLE_AUTO_ADVANCE) {
            timeout_timer = this->create_wall_timer(
                1s, std::bind(&BotoeiraNode::check_timeout, this));
        }
        
        status_timer = this->create_wall_timer(
            500ms, std::bind(&BotoeiraNode::check_robot_status, this));

        oled_update = this->create_wall_timer(1000ms, std::bind(&BotoeiraNode::update_oled_info, this));
        
        // publish_to_oled("ORIGEM:", std::to_string(botao));
        
        std::string auto_advance_status = ENABLE_AUTO_ADVANCE ? "ENABLED" : "DISABLED";
        log_manager->gera_log("Auto-advance feature: " + auto_advance_status + " (timeout: " + std::to_string(TIMEOUT_SECONDS) + "s)", LogManager::Info);
        RCLCPP_INFO(this->get_logger(), "Auto-advance: %s", auto_advance_status.c_str());
    }

private:
// Salva a fila em memória para um arquivo de texto.
    void save_queue_to_file() {
        // Garante que o diretório para o arquivo de fila exista antes de tentar escrever.
        std::filesystem::path file_path(QUEUE_FILE_PATH);
        std::filesystem::path dir_path = file_path.parent_path();
        if (!dir_path.empty() && !std::filesystem::exists(dir_path)) {
            try {
                std::filesystem::create_directories(dir_path);
                RCLCPP_INFO(this->get_logger(), "Created directory: %s", dir_path.c_str());
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Could not create directory: %s", e.what());
                log_manager->gera_log("Could not create directory: " + std::string(e.what()), LogManager::Warn);
                return;
            }
        }
        
        std::ofstream file(QUEUE_FILE_PATH);
        if (!file.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Could not open queue file for writing: %s", QUEUE_FILE_PATH.c_str());
            log_manager->gera_log("Could not open queue file for writing: " + QUEUE_FILE_PATH, LogManager::Warn);
            return;
        }
        
        file << get_current_date() << std::endl;
        
        std::queue<int> aux_queue = destination_queue;
        while (!aux_queue.empty()) {
            file << aux_queue.front() << std::endl;
            aux_queue.pop();
        }
        
        file.close();
        RCLCPP_INFO(this->get_logger(), "Queue saved to file: %s", QUEUE_FILE_PATH.c_str());
        log_manager->gera_log("Queue saved to file with " + std::to_string(destination_queue.size()) + " destinations", LogManager::Info);
    }
    
    // Carrega a fila do arquivo para a memória.
    void load_queue_from_file() {
        // 1. Tenta abrir o arquivo 'queue.txt'. Se não conseguir (porque não existe),
        //    a função simplesmente termina. Nada a fazer.
        std::ifstream file(QUEUE_FILE_PATH);
        if (!file.is_open()) {
            RCLCPP_INFO(this->get_logger(), "No queue file found at: %s", QUEUE_FILE_PATH.c_str());
            return;
        }
        
        // 2. Lê a PRIMEIRA linha do arquivo. Por convenção, essa linha DEVE conter a data
        //    em que o arquivo foi salvo.
        std::string saved_date;
        std::getline(file, saved_date);
        
        // 3. Obtém a data atual usando a função get_current_date().
        std::string current_date = get_current_date();
        
        //    Compara a data salva no arquivo com a data atual.
        if (saved_date != current_date) {
            // 5. SE AS DATAS FOREM DIFERENTES:
            // 5a. Emite um aviso no console, informando o motivo do descarte.
            RCLCPP_WARN(this->get_logger(), "Queue file is from a different day (%s vs %s). Discarding old queue.", saved_date.c_str(), current_date.c_str());
            log_manager->gera_log("Queue file is from " + saved_date + " (today is " + current_date + "). Discarding old queue.", LogManager::Warn);
            // 5b. Fecha o arquivo que estava sendo lido.
            file.close();
            // 5c. APAGA o arquivo 'queue.txt' do disco. Isso efetivamente apaga todos os destinos antigos.
            std::remove(QUEUE_FILE_PATH.c_str());
            // 5d. Termina a função imediatamente. Nenhum destino antigo é carregado para a memória.
            return;
        }
        
        // 6. SE AS DATAS FOREM IGUAIS:
        //    O código continua, lendo o resto do arquivo linha por linha e adicionando
        //    os destinos à fila em memória (destination_queue).
        std::string line;
        int count = 0;
        while (std::getline(file, line)) {
            if (!line.empty()) {
                try {
                    int destination = std::stoi(line);
                    destination_queue.push(destination);
                    count++;
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "Invalid destination value in queue file: %s", line.c_str());
                }
            }
        }
        
        file.close();
        RCLCPP_INFO(this->get_logger(), "Queue loaded from file: %d destinations restored", count);
        log_manager->gera_log("Queue loaded from file. Restored " + std::to_string(count) + " destinations from same day", LogManager::Info);
    }

private:
    void total_rotas_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        quantidade_rotas = msg->data;
        log_manager->gera_log("Total number of routes set to: " + std::to_string(quantidade_rotas), LogManager::Info);
        RCLCPP_INFO(this->get_logger(), "Total number of routes received: %d", quantidade_rotas);
    }

    void lora_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        int new_destination = msg->data;

        // HOME MUDOU

        // if (destination == HOME && msg->data == 4) { // se robo esta na home e recebe a request para ir para o ponto da home
        //     accepted_pub->publish(std_msgs::msg::Bool().set__data(false));
        //     RCLCPP_INFO(this->get_logger(), "Robot was called at point 4 when it was already at HOME point");
        //     log_manager->gera_log("Robot was called at point 4 when it was already at HOME point.", LogManager::Info);
        //     return;
        // }

        if (destination_queue.empty()) {
            // 1.1. Se o robô NÃO está se deslocando (está ocioso)
            if (!has_started && has_deployed) { // se n houver a checagem do has_deployed e o cambão for attached na home ele vai querer ir pro ponto lora
                RCLCPP_INFO(this->get_logger(), "Fila vazia e robô ocioso. Executando chamada LoRa %d diretamente.", new_destination);
                log_manager->gera_log("Fila vazia e robô ocioso. Executando chamada LoRa " + std::to_string(new_destination) + " diretamente.", LogManager::Info);

                // Publica o destino diretamente
                destination_pub->publish(std_msgs::msg::Int32().set__data(new_destination));
                
                // Atualiza o estado do robô para iniciar a nova tarefa
                destination = new_destination;
                reached = false;
                destino_enviado = true;
                has_started = true;
                timeout_active = false;
                // has_deployed = false; // Assume que uma chamada LoRa é para buscar algo

                // Publica 'false' em /request_accepted, pois o destino não foi para a fila
                accepted_pub->publish(std_msgs::msg::Bool().set__data(false));
                return; // Finaliza a função
            } 
            // 1.2. Se o robô JÁ está se deslocando (mas a fila está vazia)
            else {
                if (new_destination == destination ){   //se já estiver indo para o destino selecionado, não adiciona na fila
                    RCLCPP_INFO(this->get_logger(), "Fila vazia e robo já está em deslocamento para o destino %d, destino não adicionado na fila!", new_destination);
                    log_manager->gera_log("Fila vazia e robo já está em deslocamento para o destino " + std::to_string(new_destination) + ", destino não adicionado na fila!", LogManager::Error);

                    accepted_pub->publish(std_msgs::msg::Bool().set__data(false));

                }
                else{   // adiciona o pedido à fila 
                    RCLCPP_INFO(this->get_logger(), "Robô ocupado, mas fila vazia. Adicionando LoRa %d como primeiro item da fila.", new_destination);
                    log_manager->gera_log("Robô ocupado, mas fila vazia. Adicionando LoRa " + std::to_string(new_destination) + " como primeiro item da fila.", LogManager::Info);

                    // Adiciona o destino à fila (que estava vazia)
                    destination_queue.push(new_destination);
                    save_queue_to_file(); // Salva o novo estado da fila
                    
                    // Publica 'true' para indicar que foi enfileirado
                    accepted_pub->publish(std_msgs::msg::Bool().set__data(true));
                    
                }
                return; // Finaliza a função
            }
        }

        // Se o código chegou aqui, a fila NÃO está vazia.

        // 3. CASO A FILA ESTEJA CHEIA (verificação para destinos NOVOS)
        // Esta verificação é feita antes da de duplicatas para otimizar.
        // Se a fila está cheia e o item já existe, a verificação de duplicata abaixo vai tratar disso.
        // Esta regra só se aplica a itens que precisariam ser adicionados.
        if (destination_queue.size() >= MAX_QUEUE_SIZE) {
            // Antes de rejeitar, verifica se o destino já está na fila.
            std::queue<int> temp_q = destination_queue;
            bool is_duplicate = false;
            while(!temp_q.empty()){
                if(temp_q.front() == new_destination) {
                    is_duplicate = true;
                    break;
                }
                temp_q.pop();
            }

            if (is_duplicate) {
                // Se é duplicado, mesmo com a fila cheia, o pedido é "aceito"
                RCLCPP_WARN(this->get_logger(), "Destino %d já está na fila (que está cheia). Pedido aceito.", new_destination);
                accepted_pub->publish(std_msgs::msg::Bool().set__data(true));
            } else {
                // Se não é duplicado e a fila está cheia, rejeita.
                RCLCPP_WARN(this->get_logger(), "Fila cheia. Rejeitando novo comando LoRa: %d", new_destination);
                log_manager->gera_log("Fila cheia. Rejeitando novo comando LoRa: " + std::to_string(new_destination), LogManager::Warn);
                accepted_pub->publish(std_msgs::msg::Bool().set__data(false));
            }
            return; // Finaliza a função
        }

        // 2. CASO A FILA JÁ TENHA ITEM, MAS TENHA ESPAÇO
        
        // 2.1. Verifica se o destino já está na fila
        std::queue<int> aux_queue = destination_queue;
        while(!aux_queue.empty()) {
            if(aux_queue.front() == new_destination) {
                RCLCPP_WARN(this->get_logger(), "Destino %d já está na fila. Pedido aceito.", new_destination);
                log_manager->gera_log("Destino " + std::to_string(new_destination) + " já está na fila. Pedido aceito.", LogManager::Warn);
                accepted_pub->publish(std_msgs::msg::Bool().set__data(true));

                return; // Finaliza a função
            }
            aux_queue.pop();
        }

        // 2.2. Se tiver espaço, não estiver na fila.
         
        if (new_destination == destination ){   //se já estiver indo para o destino selecionado, não adiciona na fila
            RCLCPP_INFO(this->get_logger(), "Robo já está em deslocamento para o destino %d, destino não adicionado na fila!", new_destination);
            log_manager->gera_log("Robo já está em deslocamento para o destino " + std::to_string(new_destination) + ", destino não adicionado na fila!", LogManager::Error);

            accepted_pub->publish(std_msgs::msg::Bool().set__data(false));

        }
        else{   // adiciona o pedido à fila 
            RCLCPP_INFO(this->get_logger(), "Comando LoRa %d adicionado à fila.", new_destination);
            log_manager->gera_log("Comando LoRa " + std::to_string(new_destination) + " adicionado à fila.", LogManager::Info);
            
            destination_queue.push(new_destination);
            save_queue_to_file();
            
            accepted_pub->publish(std_msgs::msg::Bool().set__data(true));
        }
        
    }


    void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        volt_percent = ((msg->voltage - 48.0) / (54.6 - 48.0) ) * 100;
        volt_percent = std::clamp(volt_percent, 0.0, 100.0);    // para limitar entre 0% e 100%
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(1) << volt_percent;
        // voltage_battery = std::round(msg->voltage * 10) / 10;

        voltage_battery = stream.str();
        // publish_to_oled("BaT: " + voltage_battery + "%", "DEST: " + std::to_string(destination));

        // RCLCPP_INFO(this->get_logger(), "volt_percent %.2f%% ", volt_percent);

        // if(volt_percent <= 5){ // if battery gets lower than 5% 
        //     RCLCPP_WARN(this->get_logger(), "Desligando por baixa tensão: %.2f Volts | %.2f%%.", msg->voltage, volt_percent);
        //     log_manager->gera_log("Desligando por baixa tensão: " + std::to_string(msg->voltage) + " Volts | " + std::to_string(volt_percent) + "%", LogManager::Warn);
        //     sleep(2);
        //     hardware_off->publish(std_msgs::msg::Bool().set__data(true));
        // }
    }

    void reached_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        reached = msg->data;

        if(!nav_ready) {
            nav_ready = true;
            return;
        }

        if (reached && has_started)
        {
            chegou_com_carga = !has_deployed; // a ideia é :
            
            // Estado      |    has_deployed    |   chegou_com_carga   |   solicita abertura |
            // com carga   |       false        | true (!has_deployed) |         sim         |
            // sem carga   |        true        | false (!has_deployed)|         não         |

            destino_enviado = false;
            disponivel = false;

            log_manager->gera_log("Arrived at destination: " + std::to_string(destination), LogManager::Info);
            RCLCPP_INFO(this->get_logger(), "Arrived at destination: %d", destination);
            
            // --- INÍCIO DA CORREÇÃO ---
            // VERIFICAÇÃO DE FIM DE CICLO DE TRABALHO
            // Se o robô chegou em HOME e a fila está vazia, ele terminou tudo.
            if (destination == HOME && destination_queue.empty()) {
                RCLCPP_INFO(this->get_logger(), "Ciclo de trabalho finalizado. Robo está na HOME e ocioso.");
                log_manager->gera_log("Ciclo de trabalho finalizado. Robo está na HOME e ocioso", LogManager::Info);
                
                // AÇÃO PRINCIPAL: Reseta o estado para "ocioso".
                has_started = false; 
                
                // Sinaliza o estado de ocioso/pronto para o operador.
                publish_to_oled("BaT: " + voltage_battery + "%", "DISPONIVEL");

            } else {
                // Se não for o fim do ciclo (fila vazia), apenas sinaliza a chegada no ponto.
                chegou_com_carga = !has_deployed;
                som_pub->publish(create_string_msg("Chegou"));
                cor_pub->publish(create_string_msg("Verde"));
            }
            // --- FIM DA CORREÇÃO ---
            
            // Timer de paciência só inicia em pontos de CHAMADA (quando não tem cambão preso)
            // Não inicia em pontos de entrega (has_deployed == false) nem em HOME
            if (ENABLE_AUTO_ADVANCE && destination != HOME && has_deployed) {
                arrival_time = this->now();
                timeout_active = true;
                log_manager->gera_log("Patience timer started at  " + std::to_string(destination) + " (" + std::to_string(TIMEOUT_SECONDS) + "s)", LogManager::Info);
                RCLCPP_INFO(this->get_logger(), "Patience timer started: %ds", TIMEOUT_SECONDS);
            }
        }
    }

    void deployed_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if(!reached && !msg->data) { // se n chegou e recebeu uma msg que ta desengatado, ou seja, no meio do caminho engataram algo nele
            request_deploy(true);
            request_deploy_once = false;

            return;
        }

        if(was_a_early_deploy) {
            was_a_early_deploy = false;
            return;
        }

        has_deployed = msg->data;
        request_deploy_once = false;
        
        set_rotate_to_heading_enabled(has_deployed);
        
        if(has_deployed) {
            log_manager->gera_log("Kanban deployment confirmed at point: " + std::to_string(destination), LogManager::Info);
            RCLCPP_INFO(this->get_logger(), "Kanban deployment confirmed at point: %d", destination);
            if(has_started) {
                handle_seq_button();
            }
        } else {
            // Cambão foi preso no robô - CANCELA o timer de paciência
            // Agora ele fica esperando o operador apertar um botão de destino (sem timeout)
            log_manager->gera_log("Kanban attached at point: " + std::to_string(destination), LogManager::Info);
            RCLCPP_INFO(this->get_logger(), "Kanban attached at point: %d", destination);
            
            if (timeout_active) {
                timeout_active = false;
                log_manager->gera_log("Patience timer canceled,  kanban attached, waiting for destination button", LogManager::Info);
                RCLCPP_INFO(this->get_logger(), "Timer canceled: waiting for destination button");
            }
        }
    }
    
    void set_rotate_to_heading_enabled(bool enabled) {
        if (!controller_param_client->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Controller server parameter service not available");
            log_manager->gera_log("Controller server parameter service not available", LogManager::Warn);
            return;
        }

        if(!enabled) { 
            // qnd cambao ta preso, desativa o override
            if(speed_override_timer) {
                speed_override_timer->cancel();
                speed_override_timer = nullptr;
                RCLCPP_INFO(this->get_logger(), "speed limit override deactivated.");
                nav2_msgs::msg::SpeedLimit limit;
                limit.percentage = true;
                limit.speed_limit = 50.0;
                speed_limit_pub->publish(limit);
            } 
        } else { //cambao ta solto, pode soltar bala
            speed_override_timer = this->create_wall_timer(std::chrono::milliseconds(200), 
                [this]() {
                    nav2_msgs::msg::SpeedLimit limit;
                    limit.percentage = true;
                    limit.speed_limit = 0.0;
                    speed_limit_pub->publish(limit);
                }
            );
            RCLCPP_INFO(this->get_logger(), "speed limit timer activated!");
        }
        
        auto params = std::vector<rclcpp::Parameter> {
            rclcpp::Parameter("FollowPath.use_rotate_to_heading", enabled),
            rclcpp::Parameter("goal_checker.yaw_goal_tolerance", enabled ? 0.1 : 1.0)
        };
        
        auto future = controller_param_client->set_parameters({params});
        
        std::thread([this, future = std::move(future), enabled]() mutable {
            try {
                auto results = future.get();
                bool all_ok = true;

                for(const auto& result : results) {
                    if(!result.successful) {
                        all_ok = false;
                        RCLCPP_WARN(this->get_logger(), "Failed to set a parameter: %s", result.reason.c_str());
                    }
                }
                if(all_ok) {
                    std::string state = enabled ? "ENABLED" : "DISABLED";
                    double yaw_tol = enabled ? 0.1 : 1.0;
                    RCLCPP_INFO(this->get_logger(), "rotate_to_heading %s, yaw_tolerance=%.2f (kanban %s)",  state.c_str(), yaw_tol, enabled ? "free" : "attached");
                    log_manager->gera_log("rotate_to_heading " + state + ", yaw_tolerance=" + 
                                        std::to_string(yaw_tol) + " (kanban " + 
                                        (enabled ? "free" : "attached") + ")", LogManager::Info);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception setting parameters: %s", e.what());
            }

        }).detach();
    }

    void check_robot_status() {
        if(!nav_ready) {
            return;
        }
        // if (!disponivel && once_lock && reached) {
        //     once_lock = false; 
            
        //     cor_pub->publish(create_string_msg("Roxo"));
        //     som_pub->publish(create_string_msg("Travado"));
            
        //     log_manager->gera_log("Robot locked at destination. Waiting for unlock command.", LogManager::Warn);
        //     RCLCPP_WARN(this->get_logger(), "Robot is LOCKED at destination!");
        // }
        
        // else if (disponivel && once_unlock && reached) {
        //     once_unlock = false;
            
        //     cor_pub->publish(create_string_msg("Verde"));
        //     som_pub->publish(create_string_msg("Botao"));
            
        //     log_manager->gera_log("Robot unlocked and ready for next command.", LogManager::Info);
        //     RCLCPP_INFO(this->get_logger(), "Robot UNLOCKED!");
        // }

        if(!destination_queue.empty() && reached && has_deployed && destination == HOME) {
            handle_seq_button(); // chamar a sequencia qnd esta no home e recebeu uma msg LoRa
        }

        if(reached && !has_deployed && destination != HOME && chegou_com_carga) {
            request_deploy(false);


            // std::this_thread::sleep_for(3s);

            // if(has_deployed) {
            //     handle_seq_button(); // chamar a sequencia
            // }
        }
    }

    void request_deploy(bool early) {
        if(!request_deploy_once) {
            deploy_pub->publish(std_msgs::msg::Bool().set__data(true));

            if(early) {
                log_manager->gera_log("kanban was attached while moving to pickup point. Requesting kanban deployment while going to point: " + std::to_string(destination), LogManager::Info);
                RCLCPP_INFO(this->get_logger(), "kanban was attached while moving to pickup point. Requesting kanban deployment while going to point: %d", destination);
                was_a_early_deploy = true;
            } else {
                log_manager->gera_log("Requesting kanban deployment at point: " + std::to_string(destination), LogManager::Info);
                RCLCPP_INFO(this->get_logger(), "Requesting kanban deployment at point: %d", destination);
            }
          
            request_deploy_once = true;
        }
    }

    void check_timeout() {
        // Timeout só acontece se ninguém engatou cambão (has_deployed == true)
        // Se o cambão foi engatado, o timer já foi cancelado no deployed_callback
        if (timeout_active && reached && !destino_enviado && destination != HOME && has_deployed) {
            auto elapsed = (this->now() - arrival_time).seconds();
            
            if (elapsed >= TIMEOUT_SECONDS) {
                timeout_active = false;
                
                // Ninguém prendeu cambão no robô - vai pro próximo ponto da queue ou volta pra home
                log_manager->gera_log("TIMEOUT! Kanban wasn't attached at " + std::to_string(destination) + ". Going away.", LogManager::Warn);
                RCLCPP_WARN(this->get_logger(), "Timeout! Kanban wasn't attached. Going away.");
                
                cor_pub->publish(create_string_msg("Roxo"));
                // som_pub->publish(create_string_msg("Timeout"));
                
                handle_seq_button();
            }
        }
    }

    void button_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (has_deployed) { 
            RCLCPP_INFO(this->get_logger(), "Received a non-lora button press without attaching");
            log_manager->gera_log("Received a non-lora button press without attaching.", LogManager::Info);
            return; // se ja fez o deploy, ou seja, nao tem nada attached, nao tem pq receber um botão não lora
        }

        int button_id = msg->data;
        RCLCPP_INFO(this->get_logger(), "Command received on /button topic: %d", button_id);

        if (timeout_active) {
            timeout_active = false;
            log_manager->gera_log("Timeout cancelled by manual button press", LogManager::Info);
        }
        button_id = 1; // temporario para fase 1

        switch (button_id)
        {
            case 1: 
                botao = CENTRIFUGA;
                break;
            case 2: 
                botao = DESCARTE;
                break;
            case 3: 
                botao = FORNO_UM; 
                break;
            case 4:
                botao = FORNO_DOIS; 
                break;
            default:
                log_manager->gera_log("Unknown button ID received: " + std::to_string(button_id), LogManager::Warn);
                RCLCPP_WARN(this->get_logger(), "Unknown button ID: %d", button_id);
                return;

            }
        handle_seq_button();
    }

    void get_deployment_point() {
        switch(botao) {
            case DESCARTE:
                destination = deploy_descarte[deploy_at_descarte % deploy_descarte.size()];
                deploy_at_descarte++;
                log_manager->gera_log("Deploying kanban at point: " + std::to_string(destination), LogManager::Info);
                RCLCPP_INFO(this->get_logger(), "Deploying kanban at point: %d", destination);
                break;
            case CENTRIFUGA:
                destination = deploy_centrifuga[deploy_at_centrifuga % deploy_centrifuga.size()];
                deploy_at_centrifuga++;
                log_manager->gera_log("Deploying kanban at point: " + std::to_string(destination), LogManager::Info);
                RCLCPP_INFO(this->get_logger(), "Deploying kanban at point: %d", destination);
                break;
            case FORNO_UM:
                destination = deploy_forno_um[deploy_at_forno_um % deploy_forno_um.size()];
                deploy_at_forno_um++;
                log_manager->gera_log("Deploying kanban at point: " + std::to_string(destination), LogManager::Info);
                RCLCPP_INFO(this->get_logger(), "Deploying kanban at point: %d", destination);
                break;
            case FORNO_DOIS:
                destination = deploy_forno_dois[deploy_at_forno_dois % deploy_forno_dois.size()];
                deploy_at_forno_dois++;
                log_manager->gera_log("Deploying kanban at point: " + std::to_string(destination), LogManager::Info);
                RCLCPP_INFO(this->get_logger(), "Deploying kanban at point: %d", destination);
                break;
            default:
                log_manager->gera_log("Unknown botao ID for deployment: " + std::to_string(botao), LogManager::Warn);
                RCLCPP_WARN(this->get_logger(), "Unknown botao ID for deployment: %d", botao);
                break;
        }
    }

    void handle_seq_button() {
        if(volt_percent >= 10 || !has_deployed) { // se a bateria estiver abaixo de 10%, mas o kanban ainda nao foi solto, deixa o robo entregar
            if(has_deployed) {
                if(!destination_queue.empty()) {
                    destination = destination_queue.front();
                    destination_queue.pop();
                    // Salva o novo estado da fila (com um item a menos) no arquivo.
                    save_queue_to_file(); 
                    log_manager->gera_log("Processing next destination from queue: " + std::to_string(destination), LogManager::Info);
                    RCLCPP_INFO(this->get_logger(), "Next destination from queue: %d", destination);
                } else {
                    save_queue_to_file(); // A fila está vazia, a chamada a save_queue_to_file() aqui já garante que o arquivo será salvo como vazio.
                    if(destination != HOME) { // if our queue is empty but the robot is already at the HOME point, do nothing
                        //envia robo pro ponto de descanso
                        destination = HOME;
                        log_manager->gera_log("No destinations in queue. Sending robot to HOME point: " + std::to_string(destination), LogManager::Info);
                        RCLCPP_INFO(this->get_logger(), "No destinations in queue. Sending robot to HOME point: %d", destination);
                    } //else {
                    //     if(!autocalled) {
                    //         get_deployment_point();
                    //         log_manager->gera_log("Received a deploy request directly from HOME point to: " + std::to_string(destination), LogManager::Info);
                    //         RCLCPP_INFO(this->get_logger(), "Received a deploy request directly from HOME point to: %d", destination);
                    //         has_deployed = false;
                    //     }
                    // }
                }
            } else {
                get_deployment_point();
                reached = false;
            }
            destination_pub->publish(std_msgs::msg::Int32().set__data(destination));
            reached = false;
            destino_enviado = true;
            has_started = true;
            timeout_active = false;
        } else {
            if(destination != HOME) {
                destination = HOME;
                destination_pub->publish(std_msgs::msg::Int32().set__data(destination));
                reached = false;
                destino_enviado = true;
                has_started = true;
                timeout_active = false;
                log_manager->gera_log("Battery low! Sending robot to HOME point: " + std::to_string(destination), LogManager::Warn);
                RCLCPP_WARN(this->get_logger(), "Battery low! Sending robot to HOME point: %d", destination);
            } else {
                log_manager->gera_log("Cannot proceed to next destination due to low battery: " + std::to_string(volt_percent) + "%", LogManager::Error);
                RCLCPP_ERROR(this->get_logger(), "Cannot proceed to next destination due to low battery: %.2f%%", volt_percent);
            }

        }
    }

    void publish_to_oled(const std::string& line1, const std::string& line2) {
        auto msg = std_msgs::msg::String();
        msg.data = line1 + ";" + line2;
        oled_pub->publish(msg);
    }

    std_msgs::msg::String create_string_msg(const std::string& content) {
        std_msgs::msg::String msg;
        msg.data = content;
        return msg;
    }

    void update_oled_info(){
        if (!has_started && !reached){
            if (destination == 6) {
                publish_to_oled("BaT: " + voltage_battery + "%", "DEST: " + std::to_string(5));
            } else {
                publish_to_oled("BaT: " + voltage_battery + "%", "DEST: " + std::to_string(destination));
            }
        } else if (!has_started) {
            publish_to_oled("BaT: " + voltage_battery + "%", "CHEG: " + std::to_string(destination));
        }
    }

    const bool ENABLE_AUTO_ADVANCE = true; 
    const int TIMEOUT_SECONDS = 60;    // seconds of patience at the call point

    bool reached = true;
    bool has_started = false;
    bool destino_enviado = false;
    bool disponivel = true;      
    bool once_lock = false;   
    bool once_unlock = true;    
    bool lora_message_received = false;
    bool timeout_active = false;
    bool has_deployed = true; 
    bool chegou_com_carga = false;
    bool nav_ready = false;
    bool request_deploy_once = false;
    bool was_a_early_deploy = false;
    int botao = 2;
    int destination = HOME;
    int quantidade_rotas = 6;
    int deploy_at_descarte = 0;
    int deploy_at_centrifuga = 0;
    int deploy_at_forno_um = 0;
    int deploy_at_forno_dois = 0;
    const int LORA_POINT = 6; 
    double volt_percent = 100.0;
    std::string voltage_battery = "100.0";

    std::vector<int> deploy_descarte = {1, 2};
    std::vector<int> deploy_centrifuga = {6, 7, 8};
    // std::vector<int> deploy_centrifuga = {2, 3, 4}; //teste aceleradoras
    std::vector<int> deploy_forno_um = {1, 2, 3, 4, 5, 6, 7, 8};
    std::vector<int> deploy_forno_dois = {1, 2, 3, 4, 5, 6, 7, 8};
    std::queue<int> destination_queue;

    rclcpp::Time arrival_time;

    std::shared_ptr<LogManager> log_manager;
    std::shared_ptr<rclcpp::AsyncParametersClient> controller_param_client;
    rclcpp::TimerBase::SharedPtr timeout_timer;
    rclcpp::TimerBase::SharedPtr status_timer; 
    rclcpp::TimerBase::SharedPtr oled_update;
    rclcpp::TimerBase::SharedPtr speed_override_timer;
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr total_rotas_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr button_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lora_sub;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deployed_sub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr destination_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cor_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr som_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr oled_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hardware_off;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr request_accepted;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr accepted_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deploy_pub;
    rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto botoeira_node = std::make_shared<BotoeiraNode>();
    RCLCPP_INFO(botoeira_node->get_logger(), "Iniciando botoeira_ponto_linear.cpp...");
    rclcpp::spin(botoeira_node);
    rclcpp::shutdown();
    return 0;
}