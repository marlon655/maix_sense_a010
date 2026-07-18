// Inclusão do arquivo de cabeçalho que declara a classe StopObstacle.
#include "nav_hub/stop_obstacle_node.hpp" 
// Inclusão para usar durações de tempo (ex: 100ms).
#include <chrono>
// Inclusão para usar limites numéricos, como o valor máximo de um double.
#include <limits>

// Permite o uso de sufixos de tempo como 'ms' e 's' para durações.
using namespace std::chrono_literals;

/**
 * @brief Construtor da classe StopObstacle.
 * Essa classe mantem o robo parado caso detecte colisão com obstáculos. 
 * O tópico /scan contém a distancia e angulo de cada obstáculo (coord polar), esses obstáculos passam por uma filtragem:
 *  * Todos os obstáculos com distancia fora do intervalo de colisão (stopDistance), NaN, e infinito são descartados.
 *  * Todos os obstáculos fora do intervalo agunlar são descartados. O intervalo angular de interesse é aquele que esteja a frente do robo,
 * constituindo um campo de visão de 180°.
 *  * Converter a posição dos obstáculos restantes para coordnadas cartesianas, onde a origem é o lidar.
 *  * Cria uma área retangular a frente do robo (safety zone), os obstáculos que estejam fora dessa área são descartados,
 * restando apenas os obstáculos dentro da safety_zone. 
 * A detecção de colisão é feita medindo a distância mínima de um obstáculo que esteja dentro da safety_zone
 *  e alguns pontos que compõe um pedaço da rota global a frente do robô, caso essa distancia minima seja menor 
 * que a metade da largura do robo, significa que o obstáculo está muito próximo da rota global e causará uma colisão com o robo.
 * Quando uma colisão for detectaca, será enviado velocidade zero para o tópcio /cmd_vel, até que o caminho seja desobstruido.
 */
StopObstacle::StopObstacle() : Node("stop_obstacle_node")
{
    // --- Declaração e obtenção de parâmetros ---
    // Declara os parâmetros que o nó pode receber, com valores padrão.
    // Isso permite que sejam configurados externamente (ex: em um arquivo de lançamento).
    this->declare_parameter<double>("stopDistance", 1.5);
    this->declare_parameter<double>("larguraDoRobo", 0.5);
    this->declare_parameter<double>("dist_minima_margin", 0.0);
    this->declare_parameter<double>("passo_globalPath", 0.15);
    this->declare_parameter<double>("anguloDireita", 90.0);
    this->declare_parameter<double>("anguloEsquerda", 270.0);

    // Carrega os valores dos parâmetros para as variáveis da classe.
    stopDistance_ = this->get_parameter("stopDistance").as_double();
    larguraDoRobo_ = this->get_parameter("larguraDoRobo").as_double();
    // Calcula a distância mínima de segurança: metade da largura do robô mais uma margem.
    dist_minima_ = (larguraDoRobo_ / 2.0) + this->get_parameter("dist_minima_margin").as_double();
    passo_globalPath_ = this->get_parameter("passo_globalPath").as_double();
    anguloDireita_ = this->get_parameter("anguloDireita").as_double();
    anguloEsquerda_ = this->get_parameter("anguloEsquerda").as_double();

    // --- Inicialização de TF ---
    // Cria um buffer para armazenar as transformações de coordenadas (TF).
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Cria um listener que escuta as transformações publicadas no ROS e as preenche no buffer.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- Publishers ---
    // Cria um publisher para enviar comandos de velocidade. Este nó assume o controle do /cmd_vel.
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/custom_cmd_vel", 1);

    // ---  Subscribers ---
    // Cria um subscriber para receber dados do LIDAR. Usa QoS de Sensor para evitar perda de dados.
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), std::bind(&StopObstacle::scan_callback, this, std::placeholders::_1));
    // Subscriber para receber a velocidade que o sistema de navegação principal *deseja* aplicar.
    vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_stop_obstacle", 1, std::bind(&StopObstacle::vel_callback, this, std::placeholders::_1));
    // Subscriber para receber o plano de rota global do planejador.
    plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan_smoothed", 1, std::bind(&StopObstacle::global_plan_callback, this, std::placeholders::_1));
    // Subscriber para receber a pose atual do robô (posição e orientação) do sistema de localização (AMCL).
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 1, std::bind(&StopObstacle::pose_callback, this, std::placeholders::_1));
    // Subscriber para um flag booleano que indica se o robô alcançou o destino final.
    reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "has_reached", 1, std::bind(&StopObstacle::reached_callback, this, std::placeholders::_1));

    // --- Timer principal (loop run) ---
    // Cria um timer que chama a função `run()` a cada 100ms (frequência de 10 Hz).
    // Este timer funciona como o `while not rospy.is_shutdown()` e `rate.sleep()` do ROS1.
    timer_ = this->create_wall_timer(100ms, std::bind(&StopObstacle::run, this));
    
    RCLCPP_INFO(this->get_logger(), "Nó stop_obstacle_node iniciado.");
}

/**
 * @brief Callback para mensagens do tópico /scan.
 * Armazena os dados do LIDAR e inicializa variáveis na primeira chamada.
 */
void StopObstacle::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    laser_ranges_ = msg->ranges; // Armazena o array de distâncias.
    if (!flagIniciouScan_) { // Executa apenas na primeira vez que recebe um scan.
        flagIniciouScan_ = true;
        angle_min_ = msg->angle_min; // Ângulo do primeiro feixe.
        angle_increment_ = msg->angle_increment; // Incremento angular entre feixes.
        laser_frame_id_ = msg->header.frame_id; // Nome do frame de coordenadas do LIDAR.
        // RCLCPP_INFO(this->get_logger(), "angle_min_: %.3f", angle_min_);
        // RCLCPP_INFO(this->get_logger(), "angle_increment_: %.3f", angle_increment_);
        // RCLCPP_INFO(this->get_logger(), "laser_frame_id_: %s", laser_frame_id_.c_str());
    }
}

/**
 * @brief Callback para mensagens do tópico /amcl_pose.
 * Atualiza a posição (x, y) atual do robô.
 */
void StopObstacle::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
}

/**
 * @brief Callback para mensagens do tópico /cmd_vel_stop_obstacle.
 * Armazena o comando de velocidade desejado pelo sistema de navegação.
 */
void StopObstacle::vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    velocity_ = *msg;
}

/**
 * @brief Callback para o tópico has_reached.
 * Reseta o estado da rota quando o destino é alcançado.
 */
void StopObstacle::reached_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    reached_ = msg->data;
    if (!started_) { // Lógica de inicialização na primeira chamada.
        started_ = true;
    }
    if (reached_) { // Se o destino foi alcançado.
        iniciouRota_ = false; // Marca que a rota atual terminou.
    }
}

/**
 * @brief Callback para o tópico /plan_smoothed.
 * Armazena a nova rota global quando uma nova navegação começa.
 */
void StopObstacle::global_plan_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!iniciouRota_) { // Só aceita uma nova rota se a anterior terminou.
        rotaInicial_ = msg; // Armazena o plano.
        LastIndexRout_ = 0; // Reseta o índice de acompanhamento da rota.
        iniciouRota_ = true; // Marca que uma nova rota está ativa.
        RCLCPP_INFO(this->get_logger(), "Nova rota global recebida com %zu poses.", msg->poses.size());
        
        // --- LÓGICA PARA CALCULAR A DISTÂNCIA MÉDIA ENTRE PONTOS ---
        if (msg->poses.size() > 1) {
            double total_distance = 0.0;
            int num_segments = 0;
            // Itera sobre os segmentos da rota para somar as distâncias.
            for (size_t i = 0; i < msg->poses.size() - 1; ++i) {
                double dist = euclideanDistance(
                    msg->poses[i].pose.position.x, msg->poses[i].pose.position.y,
                    msg->poses[i+1].pose.position.x, msg->poses[i+1].pose.position.y
                );
                // Ignora segmentos de distância zero que podem ocorrer.
                if (dist > 1e-6) {
                    total_distance += dist;
                    num_segments++;
                }
            }

            // Calcula a média e armazena na variável da classe.
            if (num_segments > 0) {
                dist_entre_pontos_ = total_distance / num_segments;
                RCLCPP_INFO(this->get_logger(), "Distância média entre pontos da rota calculada: %.4f metros", dist_entre_pontos_);
            } else {
                // Caso todos os pontos sejam coincidentes, usa um valor padrão seguro.
                dist_entre_pontos_ = 0.05;
                RCLCPP_WARN(this->get_logger(), "Não foi possível calcular a distância média. Usando valor padrão: %.2f", dist_entre_pontos_);
            }
        } else {
            // Se a rota tem 0 ou 1 ponto, não há distância a calcular.
            dist_entre_pontos_ = 0.05;
            RCLCPP_WARN(this->get_logger(), "Rota com menos de 2 pontos. Usando distância padrão: %.2f", dist_entre_pontos_);
        }
    }
}

/**
 * @brief Calcula a distância euclidiana entre dois pontos.
 */
double StopObstacle::euclideanDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

/**
 * @brief Encontra o índice do ponto na rota global que está mais próximo da posição atual do robô.
 * Isso evita que o robô verifique partes da rota que já passaram (trimming na rota global).
 */
size_t StopObstacle::positionAtRout() {
    if (!rotaInicial_ || rotaInicial_->poses.empty() || LastIndexRout_ >= rotaInicial_->poses.size()) {
        return LastIndexRout_;
    }

    double dist_minima_robot_rout = std::numeric_limits<double>::infinity();
    size_t indice = LastIndexRout_;

    // Procura a partir do último índice conhecido para otimizar.
    for (size_t i = LastIndexRout_; i < rotaInicial_->poses.size(); ++i) {
        double dist = euclideanDistance(robot_x_, robot_y_, rotaInicial_->poses[i].pose.position.x, rotaInicial_->poses[i].pose.position.y);
        if (dist < dist_minima_robot_rout) {
            dist_minima_robot_rout = dist;
            indice = i;
        }
    }
    return indice;
}

/**
 * @brief Filtra os dados do LIDAR para encontrar obstáculos relevantes, iterando nos quadrantes frontais definidos por parâmetros.
 * 1. Usa anguloDireita_ para definir o limite do primeiro quadrante (ex: 90°).
 * 2. Usa anguloEsquerda_ para definir o início do segundo quadrante (ex: 270°).
 * 3. Itera em dois loops separados (0 a anguloDireita) e (anguloEsquerda a 360).
 * 4. Dentro dos loops, aplica todos os filtros (distância, validade, conversão e caixa de segurança).
 * @return Um vetor de pares (x, y) com as coordenadas dos obstáculos no frame do lidar.
 */
std::vector<std::pair<double, double>> StopObstacle::get_front_obstacles() {
    std::vector<std::pair<double, double>> obstacles;
    // Garante que temos dados do scan e que o incremento angular não é zero para evitar divisão por zero.
    if (laser_ranges_.empty() || angle_increment_ == 0.0) {
        return obstacles;
    }

    // Calcula os índices correspondentes aos ângulos de interesse usando as variáveis anguloDireita_e  anguloEsquerda_

    // Converte os ângulos de graus para radianos.
    double end_angle_q1_rad = anguloDireita_ * M_PI / 180.0;   // Ex: 90 graus
    double start_angle_q2_rad = anguloEsquerda_ * M_PI / 180.0; // Ex: 270 graus

    // Calcula o índice final para o primeiro loop (0° até anguloDireita_).
    size_t end_index_q1 = 0;
    if (end_angle_q1_rad > angle_min_) {
        end_index_q1 = static_cast<size_t>((end_angle_q1_rad - angle_min_) / angle_increment_);
    }

    // Calcula o índice inicial para o segundo loop (anguloEsquerda_ até o fim).
    size_t start_index_q2 = 0;
    if (start_angle_q2_rad > angle_min_) {
        start_index_q2 = static_cast<size_t>((start_angle_q2_rad - angle_min_) / angle_increment_);
    }

    // Garante que os índices não ultrapassem os limites do vetor de scans.
    if (end_index_q1 >= laser_ranges_.size()) {
        end_index_q1 = laser_ranges_.size() - 1;
    }
    if (start_index_q2 >= laser_ranges_.size()) {
        start_index_q2 = laser_ranges_.size() - 1;
    }

    // Função auxiliar (lambda) para processar um feixe e evitar duplicação de código.
    auto process_range = [&](size_t i) {
        // Ignora leituras inválidas (Inf, NaN) ou que estão além da distância de interesse.
        double range = laser_ranges_[i];
        if (!std::isfinite(range) || range >= stopDistance_) {
            return; // Pula para a próxima iteração do loop.
        }

        // Converte a leitura polar (range, angle) para cartesiana (x, y).
        double angle = angle_min_ + i * angle_increment_;
        double x = range * std::cos(angle);
        double y = range * std::sin(angle);

        // Aplica um filtro de "caixa de segurança" retangular.
        if (x > 0.0 && x < stopDistance_ && std::abs(y) < dist_minima_) {
            obstacles.push_back({x, y});        //  adicionar um novo elemento ao final do vetor.
        }
    };

    // ---Iterar no primeiro quadrante frontal (0° até anguloDireita_) ---
    for (size_t i = 0; i <= end_index_q1; ++i) {
        process_range(i);
    }

    // ---Iterar no segundo quadrante frontal (anguloEsquerda_ até 360°) ---
    for (size_t i = start_index_q2; i < laser_ranges_.size(); ++i) {
        process_range(i);
    }

    // --- Retorna a lista de obstáculos encontrados ---
    return obstacles;
}


/**
 * @brief Transforma um ponto de obstáculo do frame do LIDAR para o frame global "map".
 * @return Um `std::optional` contendo o par (x, y) global se a transformação for bem-sucedida.
 */
std::optional<std::pair<double, double>> StopObstacle::lidar_to_global(double lidar_x, double lidar_y) {
    if (laser_frame_id_.empty()) return std::nullopt;

    geometry_msgs::msg::PointStamped point_local;
    point_local.header.frame_id = laser_frame_id_;
    point_local.header.stamp = this->get_clock()->now();
    point_local.point.x = lidar_x;
    point_local.point.y = lidar_y;
    point_local.point.z = 0.0;

    try {
        // Usa o buffer de TF para transformar o ponto.
        geometry_msgs::msg::PointStamped point_global = tf_buffer_->transform(point_local, "map", tf2::durationFromSec(0.2));
        return std::make_pair(point_global.point.x, point_global.point.y);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Falha ao transformar coordenadas do scan para o mapa: %s", ex.what());
        return std::nullopt;
    }
}

/**
 * @brief Função principal de detecção. Compara a posição dos obstáculos com a rota global.
 * @return `true` se um obstáculo está próximo o suficiente da rota global para causar uma colisão, `false` caso contrário.
 */
bool StopObstacle::comparePosition() {
    // 1. Obtém os obstáculos na zona de segurança.
    auto obstacles = get_front_obstacles();

    if (obstacles.empty() || !rotaInicial_) return false;

    // 2. Converte as coordenadas dos obstáculos (frame do lidar) para o frame global "map".
    std::vector<std::pair<double, double>> coordenadasGlobais;
    for (const auto& obs : obstacles) {
        auto global_coords = lidar_to_global(obs.first, obs.second);
        if (global_coords) {
            coordenadasGlobais.push_back(*global_coords);   //  adicionar um novo elemento ao final do vetor.
        }
    }
    if (coordenadasGlobais.empty()) return false;

    // --- Loop para imprimir as coordenadas de cada obstáculo ---
    // Este log pode ser muito verboso. Considere usar RCLCPP_DEBUG ou comentá-lo em produção.
    // RCLCPP_INFO(this->get_logger(), "--- Obstáculos na Safety Zone (%zu) ---", coordenadasGlobais.size());
    // for (const auto& obs : coordenadasGlobais) {
    //     RCLCPP_INFO(this->get_logger(), "  - Obstáculo local em (x: %.3f, y: %.3f)", obs.first, obs.second);
    // }
    

    // 3. Encontra o ponto mais próximo na rota para começar a verificação.
    LastIndexRout_ = positionAtRout();

    // Otimização: calcula a cada quantos pontos da rota a verificação será feita.
    // calcule de quantos em quantos índices eu devo pular para ser eficiente. Se não for seguro fazer esse cálculo, apenas me diga para verificar todos os índices."
    double step_ratio = (dist_entre_pontos_ > 1e-6) ? std::round(passo_globalPath_ / dist_entre_pontos_) : 1.0;
    if (step_ratio < 1.0) step_ratio = 1.0;

    // Otimização: define um limite máximo de verificação na rota para não ir muito longe.
    // Calcule o índice na rota global que corresponde ao limite máximo (stopDistance_), começando da posição atual do robô, para que eu não perca tempo verificando colisões contra partes da rota que estão muito distantes para serem relevantes."
    size_t max_check_index = LastIndexRout_ + static_cast<size_t>(stopDistance_ / dist_entre_pontos_);
    if (max_check_index > rotaInicial_->poses.size()) {
        max_check_index = rotaInicial_->poses.size();
    }

    // 4. Itera sobre cada obstáculo e compara com os pontos da rota.
    for (const auto& coord : coordenadasGlobais) {
        for (size_t i = LastIndexRout_; i < max_check_index; ++i) {
            // Verifica apenas a cada 'step_ratio' pontos para otimizar.
            if (static_cast<int>(i - LastIndexRout_) % static_cast<int>(step_ratio) == 0) {
                const auto& pose = rotaInicial_->poses[i];
                double dist = euclideanDistance(coord.first, coord.second, pose.pose.position.x, pose.pose.position.y);
                // Se a distância for menor que a mínima, há risco de colisão.
                if (dist < dist_minima_) {
                    return true; // Obstáculo encontrado na rota.
                }
            }
        }
    }
    return false; // Nenhum obstáculo encontrado na rota.
}

/**
 * @brief Gera uma transição de velocidade suave de um valor inicial para um final.
 * Publica comandos de velocidade intermediários para evitar paradas e arranques bruscos.
 */
geometry_msgs::msg::Twist StopObstacle::smooth_transition(geometry_msgs::msg::Twist vel_inicial, geometry_msgs::msg::Twist vel_final, double duration) {
    int freq_pub = 600; // Alta frequência para uma transição suave.
    int steps = static_cast<int>(duration * freq_pub);
    if (steps < 1) {
        cmd_vel_pub_->publish(vel_final);
        return vel_final;
    }

    double delta_lin_x = vel_final.linear.x - vel_inicial.linear.x;
    double delta_ang_z = vel_final.angular.z - vel_inicial.angular.z;

    geometry_msgs::msg::Twist twist;
    auto sleep_duration = std::chrono::microseconds(1000000 / freq_pub);

    for (int i = 1; i <= steps; ++i) {
        double alpha = static_cast<double>(i) / steps; // Progresso da transição (0 a 1).
        // Interpolação quadrática: começa rápido e desacelera no final.
        double interpolation = 1.0 - std::pow(1.0 - alpha, 2);

        twist.linear.x = vel_inicial.linear.x + delta_lin_x * interpolation;
        twist.angular.z = vel_inicial.angular.z + delta_ang_z * interpolation;

        // Condição de parada antecipada se a velocidade for muito próxima de zero.
        if (std::abs(twist.linear.x) < 0.004 && std::abs(twist.angular.z) < 0.01) {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            cmd_vel_pub_->publish(twist);
            return twist;
        }

        cmd_vel_pub_->publish(twist);
        rclcpp::sleep_for(sleep_duration); // Pequena pausa entre publicações.
    }
    cmd_vel_pub_->publish(vel_final); // Garante que a velocidade final exata seja publicada.
    return vel_final;
}

/**
 * @brief Função principal do nó, chamada periodicamente pelo timer.
 * Orquestra a lógica de verificação de obstáculos e controle de velocidade.
 */
void StopObstacle::run() {
    // Não faz nada se não houver uma rota ativa ou se o LIDAR ainda não foi recebido.
    if (!iniciouRota_ || !flagIniciouScan_) {
        return;
    }

    // Verifica se há um obstáculo na rota.
    bool obstacle_found = comparePosition();
    
    if (obstacle_found) { // Se um obstáculo foi encontrado...
        if (!FlagIniciouRotina_) {
            FlagIniciouRotina_ = true;
            RCLCPP_INFO(this->get_logger(), "Iniciou stop_obstacle_node.");
        }
        // Se o robô estava se movendo, inicia uma transição suave para parar.
        // if (lastVel_.linear.x > 0.0 || std::abs(lastVel_.angular.z) > 0.0) {
        //     lastVel_ = smooth_transition(lastVel_, geometry_msgs::msg::Twist(), 1.0);
        // } else { // Se já estava parado, apenas continua publicando velocidade zero.
        //     cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
        // }
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist()); // publicando velocidade zero.
    } else { // Se o caminho está livre...
        if (FlagIniciouRotina_) {
            FlagIniciouRotina_ = false;
            RCLCPP_INFO(this->get_logger(), "Finalizou stop_obstacle_node.");
        }
        // Repassa o comando de velocidade desejado pelo sistema de navegação.
        // lastVel_ = velocity_;
        
        cmd_vel_pub_->publish(velocity_);
    }
}

/**
 * @brief Função main, ponto de entrada do programa.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Inicializa o ROS2.
    auto node = std::make_shared<StopObstacle>(); // Cria o nó.
    rclcpp::spin(node); // Mantém o nó vivo, processando callbacks.
    rclcpp::shutdown(); // Encerra o ROS2.
    return 0;
}
