#include <fstream>
#include <cmath>
#include <memory>
#include <string>

#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

#include "nav_hub/json.hpp"

using ordered_json = nlohmann::ordered_json;

class SimMainRouteGraph : public rclcpp::Node
{
public:
    SimMainRouteGraph() : Node("sim_main_route_graph")
    {
        route_file_ = declare_parameter<std::string>("route_file", "");
        publish_initial_pose_ = declare_parameter<bool>("publish_initial_pose", false);
        enable_status_topics_ = declare_parameter<bool>("enable_status_topics", true);

        auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        destination_sub_ = create_subscription<std_msgs::msg::Int32>(
            "destination",
            latched_qos,
            std::bind(&SimMainRouteGraph::destination_callback, this, std::placeholders::_1));

        nav_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
            "navigate_to_pose/_action/status",
            10,
            std::bind(&SimMainRouteGraph::nav_status_callback, this, std::placeholders::_1));

        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
        reached_pub_ = create_publisher<std_msgs::msg::Bool>("has_reached", latched_qos);
        navigating_pub_ = create_publisher<std_msgs::msg::Bool>("navegando", 10);

        if (!load_route_file()) {
            RCLCPP_WARN(get_logger(), "sim_main_route_graph iniciou sem arquivo de rotas valido.");
        }

        publish_status(false, true);
        RCLCPP_INFO(get_logger(), "sim_main_route_graph pronto.");
    }

private:
    bool load_route_file()
    {
        if (route_file_.empty()) {
            RCLCPP_ERROR(get_logger(), "Parametro route_file esta vazio.");
            return false;
        }

        std::ifstream file(route_file_);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Nao foi possivel abrir route_file: %s", route_file_.c_str());
            return false;
        }

        try {
            file >> route_data_;
        } catch (const std::exception& error) {
            RCLCPP_ERROR(get_logger(), "Erro lendo route_file: %s", error.what());
            return false;
        }

        RCLCPP_INFO(get_logger(), "Arquivo de destinos carregado: %s", route_file_.c_str());
        return true;
    }

    bool get_pose_for_sequence(int sequence, geometry_msgs::msg::PoseStamped& pose) const
    {
        if (route_data_.empty()) {
            return false;
        }

        for (const auto& item : route_data_.items()) {
            const auto& data = item.value();
            if (!data.is_object() || data.value("sequence", -1) != sequence) {
                continue;
            }

            pose.header.frame_id = "map";
            pose.header.stamp = now();
            pose.pose.position.x = data.value("x", 0.0);
            pose.pose.position.y = data.value("y", 0.0);
            pose.pose.position.z = 0.0;
            pose.pose.orientation.z = data.value("z", 0.0);
            pose.pose.orientation.w = data.value("w", 1.0);

            if (std::abs(pose.pose.orientation.z) < 1e-9 &&
                std::abs(pose.pose.orientation.w) < 1e-9) {
                pose.pose.orientation.w = 1.0;
            }

            return true;
        }

        return false;
    }

    void publish_initial_pose_once()
    {
        if (!publish_initial_pose_ || initial_pose_published_ || !route_data_.contains("initial")) {
            return;
        }

        const auto& initial = route_data_["initial"];
        auto pose = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose.header.frame_id = "map";
        pose.header.stamp = now();
        pose.pose.pose.position.x = initial.value("x", 0.0);
        pose.pose.pose.position.y = initial.value("y", 0.0);
        pose.pose.pose.position.z = 0.0;
        pose.pose.pose.orientation.z = initial.value("z", 0.0);
        pose.pose.pose.orientation.w = initial.value("w", 1.0);

        if (initial.contains("covariance") && initial["covariance"].is_array()) {
            const auto& covariance = initial["covariance"];
            for (size_t i = 0; i < covariance.size() && i < 36; ++i) {
                pose.pose.covariance[i] = covariance[i].get<double>();
            }
        }

        initial_pose_pub_->publish(pose);
        initial_pose_published_ = true;
        RCLCPP_INFO(get_logger(), "Pose inicial publicada a partir do JSON.");
    }

    void publish_status(bool navigating, bool reached)
    {
        if (!enable_status_topics_) {
            return;
        }

        navigating_pub_->publish(std_msgs::msg::Bool().set__data(navigating));
        reached_pub_->publish(std_msgs::msg::Bool().set__data(reached));
    }

    void destination_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (route_data_.empty() && !load_route_file()) {
            return;
        }

        publish_initial_pose_once();

        geometry_msgs::msg::PoseStamped goal;
        if (!get_pose_for_sequence(msg->data, goal)) {
            RCLCPP_ERROR(get_logger(), "Destino sequence=%d nao encontrado em %s", msg->data, route_file_.c_str());
            return;
        }

        goal_pub_->publish(goal);
        publish_status(true, false);
        RCLCPP_INFO(
            get_logger(),
            "Destino %d publicado em /goal_pose: x=%.3f y=%.3f",
            msg->data,
            goal.pose.position.x,
            goal.pose.position.y);
    }

    void nav_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
    {
        if (msg->status_list.empty()) {
            return;
        }

        const auto status = msg->status_list.back().status;
        if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
            publish_status(false, true);
            RCLCPP_INFO(get_logger(), "Navegacao concluida.");
        } else if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
                   status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
            publish_status(false, false);
            RCLCPP_WARN(get_logger(), "Navegacao abortada ou cancelada.");
        }
    }

    std::string route_file_;
    ordered_json route_data_;
    bool publish_initial_pose_ = true;
    bool enable_status_topics_ = true;
    bool initial_pose_published_ = false;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr destination_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr navigating_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimMainRouteGraph>());
    rclcpp::shutdown();
    return 0;
}
