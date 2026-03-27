#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "rover_messages/msg/drive_move_cmd.hpp"

// This file is used with GAZEBO!

class SimBridge : public rclcpp::Node {
public:
    SimBridge() : Node("sim_bridge") {
        move_cmd_sub_ = this->create_subscription<rover_messages::msg::DriveMoveCmd>(
            "rover/drive/drive_move_cmd", 10,
            std::bind(&SimBridge::move_cmd_callback, this, std::placeholders::_1));
        
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Simulation bridge started - converting DriveMoveCmd to Twist");
    }

private:
    void move_cmd_callback(const rover_messages::msg::DriveMoveCmd::SharedPtr msg) {
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = msg->x_cmd;
        twist.linear.y = msg->y_cmd;  // In case you use omnidirectional movement
        twist.angular.z = msg->yaw_cmd;
        
        cmd_vel_pub_->publish(twist);
    }

    rclcpp::Subscription<rover_messages::msg::DriveMoveCmd>::SharedPtr move_cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimBridge>());
    rclcpp::shutdown();
    return 0;
}