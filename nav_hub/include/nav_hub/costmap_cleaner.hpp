#ifndef NAV_HUB_COSTMAP_CLEANER_HPP
#define NAV_HUB_COSTMAP_CLEANER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief Considering the changes made by the nav2 in the clear costmap service (comparing to ROS 1), where now we need to create two services, one for the global costmap and other for the local, I made this class to make it easier to call these services.
 */
class CostmapCleaner : public rclcpp::Node {
public:
    CostmapCleaner();
    ~CostmapCleaner() = default;

    /** 
     * @brief Clear both local and global costmaps entirely
    */
    void clearAllCostmaps();

    /**
     * @brief Clear only local costmap entirely
     */
    void clearEntireLocalCostmap();

    /**
     * @brief Clear only global costmap entirely
     */
    void clearGlobalCostmap();

    /**
     * @brief clear all costmaps around the robot
     */
    void clearCostmapsAroundRobot(int meters = 5);

    /**
     * @brief clear local costmap around the robot
     */
    void clearAroundLocalCostmap(int meters = 5);
    /**
     * @brief clear global costmap around the robot
     */
    void clearAroundGlobalCostmap(int meters = 5);

private:
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_client;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_client;

    rclcpp::Client<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr clear_around_local_client;
    rclcpp::Client<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr clear_around_global_client;

    void callClearService(rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client, 
        const std::string& service_name);

    void callClearAroundService(rclcpp::Client<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr client,
        const std::string& service_name, int meters);
};

#endif // NAV_HUB_COSTMAP_CLEANER_HPP