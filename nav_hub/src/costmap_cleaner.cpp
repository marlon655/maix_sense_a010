#include "nav_hub/costmap_cleaner.hpp"

CostmapCleaner::CostmapCleaner() : Node("costmap_cleaner") {
    clear_local_client = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
        "local_costmap/clear_entirely_local_costmap");
    clear_global_client = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
        "global_costmap/clear_entirely_global_costmap");
    
    clear_around_local_client = this->create_client<nav2_msgs::srv::ClearCostmapAroundRobot>(
        "local_costmap/clear_around_local_costmap");
    clear_around_global_client = this->create_client<nav2_msgs::srv::ClearCostmapAroundRobot>(
        "global_costmap/clear_around_global_costmap");
}

void CostmapCleaner::clearAllCostmaps() {
    RCLCPP_INFO(this->get_logger(), "Clearing all costmaps..");
    clearEntireLocalCostmap();
    clearGlobalCostmap();
    RCLCPP_INFO(this->get_logger(), "All costmaps cleared!");
}

void CostmapCleaner::clearEntireLocalCostmap() {
    callClearService(clear_local_client, "local_costmap");
}

void CostmapCleaner::clearGlobalCostmap() {
    callClearService(clear_global_client, "global_costmap");
}

void CostmapCleaner::clearCostmapsAroundRobot(int meters) {
    RCLCPP_INFO(this->get_logger(), "Clearing costmaps around robot (%d meters)..", meters);
    clearAroundLocalCostmap(meters);
    clearAroundGlobalCostmap(meters);
    RCLCPP_INFO(this->get_logger(), "Costmaps around robot cleared!");
}

void CostmapCleaner::clearAroundLocalCostmap(int meters) {
    callClearAroundService(clear_around_local_client, "local_costmap", meters);
}

void CostmapCleaner::clearAroundGlobalCostmap(int meters) {
    callClearAroundService(clear_around_global_client, "global_costmap", meters);
}

void CostmapCleaner::callClearService(rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client, const std::string& service_name) {
    while(!client->wait_for_service(1s)) {
        if(!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Service interrupted while waiting for %s clear service", service_name.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for %s clear service to be available...", service_name.c_str());
    }

    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    auto promise = client->async_send_request(request);

    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), promise, 5s) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s cleared successfully.", service_name.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to clear %s", service_name.c_str());
    }
}

void CostmapCleaner::callClearAroundService(rclcpp::Client<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr client, const std::string& service_name, int meters) {
    while(!client->wait_for_service(1s)) {
        if(!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Service interrupted while waiting for %s clear around service", service_name.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for %s clear around service to be available...", service_name.c_str());
    }

    auto request = std::make_shared<nav2_msgs::srv::ClearCostmapAroundRobot::Request>();
    request->reset_distance = meters;
    
    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "%s clear around robot completed", service_name.c_str());
    } else {
        RCLCPP_ERROR(get_logger(), "%s clear around robot failed", service_name.c_str());
    }
}