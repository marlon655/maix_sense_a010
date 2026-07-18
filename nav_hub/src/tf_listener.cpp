#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>

class TFListener : public rclcpp::Node
{
public:
    TFListener() : Node("tf_listener_node")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "map_pose", 1
        );
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20Hz = 50ms
            std::bind(&TFListener::run, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "TF Listener initialized!");
    }

private:
    void run()
    {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                "map", "base_footprint", tf2::TimePointZero
            );
            
            auto position = geometry_msgs::msg::PoseWithCovarianceStamped();
            position.header.stamp = this->get_clock()->now();
            position.header.frame_id = "map";
            
            position.pose.pose.position.x = transform_stamped.transform.translation.x;
            position.pose.pose.position.y = transform_stamped.transform.translation.y;
            position.pose.pose.position.z = transform_stamped.transform.translation.z;
            
            position.pose.pose.orientation.x = transform_stamped.transform.rotation.x;
            position.pose.pose.orientation.y = transform_stamped.transform.rotation.y;
            position.pose.pose.orientation.z = transform_stamped.transform.rotation.z;
            position.pose.pose.orientation.w = transform_stamped.transform.rotation.w;
            
            pose_publisher_->publish(position);
            
        } catch (const tf2::TransformException& ex) {
            // RCLCPP_DEBUG(this->get_logger(), "Transform exception: %s", ex.what());
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TFListener>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
