#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>
#include "rover_messages/msg/drive_move_cmd.hpp"
#include "rover_messages/msg/drive_op_cmd.hpp"
#include "rover_messages/msg/auton_drive.hpp"
#include "rover_messages/Enums.hpp"

class GoalDriver : public rclcpp::Node {
private:
    // Velocity limits
    double max_linear_vel = 1;    // Maximum forward/backward velocity (m/s)
    double max_angular_vel = 1.0;   // Maximum rotation velocity (rad/s)
    
    // Goal parameters
    double goal_timeout_sec = 30.0; // Maximum time to reach goal before giving up (seconds)
    double goal_tolerance = 0.1;    // How close to goal counts as "reached" (meters)
    double turn_threshold = 0.5;    // Angular error threshold to decide turn-in-place vs drive (~28 degrees in radians)
    double max_distance = 2.0;      // Distance at which velocity is fully scaled (meters)
    
public:
    GoalDriver() : Node("goal_driver"), 
                   goal_received_(false),
                   current_mode_(DriveMode::NPT) {
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Subscribe to goal pose (RViz or nav2 style)
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&GoalDriver::goal_callback, this, std::placeholders::_1));
        
        // Subscribe to direct position commands
        drive_pos_ = this->create_subscription<rover_messages::msg::AutonDrive>(
            "/drive_pos", 10,
            std::bind(&GoalDriver::pos_callback, this, std::placeholders::_1));
        
        move_cmd_pub_ = this->create_publisher<rover_messages::msg::DriveMoveCmd>(
            "rover/drive/move_cmd", 10);
        
        op_cmd_pub_ = this->create_publisher<rover_messages::msg::DriveOpCmd>(
            "rover/drive/op_cmd", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GoalDriver::control_loop, this));
        
        // Set initial drive mode to NPT
        send_drive_mode(DriveMode::NPT);
        
        RCLCPP_INFO(this->get_logger(), 
                    "Goal driver started\n"
                    "  Max linear vel: %.2f m/s\n"
                    "  Max angular vel: %.2f rad/s\n"
                    "  Goal tolerance: %.2f m\n"
                    "  Turn threshold: %.2f rad (%.1f deg)\n"
                    "  Max distance for normalization: %.2f m\n", 
                    max_linear_vel, max_angular_vel, goal_tolerance,
                    turn_threshold, turn_threshold * 180.0 / M_PI, max_distance);
    }

private:
    void send_drive_mode(DriveMode mode) {
        // Only send if mode actually changed (avoid spam)
        if (mode == current_mode_) return;
        
        auto op_cmd = rover_messages::msg::DriveOpCmd();
        op_cmd.drive_mode = static_cast<uint8_t>(mode);
        op_cmd.drive_limit = static_cast<uint8_t>(DriveLimits::NO_UPDATE);
        op_cmd_pub_->publish(op_cmd);
        
        current_mode_ = mode;
        
        const char* mode_name = (mode == DriveMode::NPT) ? "NPT" :
                               (mode == DriveMode::LINEAR) ? "LINEAR" :
                               (mode == DriveMode::ACKERMANN) ? "ACKERMANN" :
                               (mode == DriveMode::CRABERMANN) ? "CRABERMANN" : "UNKNOWN";
        
        RCLCPP_INFO(this->get_logger(), "Drive mode changed to: %s", mode_name);
    }
    
    void emergency_stop() {
        auto cmd = rover_messages::msg::DriveMoveCmd();
        cmd.x_cmd = 0.0;
        cmd.y_cmd = 0.0;
        cmd.yaw_cmd = 0.0;
        move_cmd_pub_->publish(cmd);
        goal_received_ = false;
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP ACTIVATED");
    }
    
    void pos_callback(const rover_messages::msg::AutonDrive::SharedPtr msg) {
        // Check if stop command
        if (msg->stop) {
            RCLCPP_WARN(this->get_logger(), "Stop command received via /drive_pos");
            emergency_stop();
            return;
        }
        
        // Validate position values (check for NaN or Inf)
        if (!std::isfinite(msg->x_cmd) || !std::isfinite(msg->y_cmd)) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Invalid position received via /drive_pos! Contains NaN or Inf values. Stopping.");
            emergency_stop();
            return;
        }
        
        // Set new goal in odom frame
        goal_x_ = msg->x_cmd;
        goal_y_ = msg->y_cmd;
        goal_received_ = true;
        goal_start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "New goal from /drive_pos: (%.2f, %.2f)", goal_x_, goal_y_);
    }
    
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Validate goal pose (check for NaN or Inf)
        if (!std::isfinite(msg->pose.position.x) || 
            !std::isfinite(msg->pose.position.y) ||
            !std::isfinite(msg->pose.orientation.z) ||
            !std::isfinite(msg->pose.orientation.w)) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Invalid goal pose received! Contains NaN or Inf values. Stopping.");
            emergency_stop();
            return;
        }
        
        // Set new goal in odom frame
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        goal_received_ = true;
        goal_start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "New goal from /goal_pose: (%.2f, %.2f)", goal_x_, goal_y_);
    }
    
    void control_loop() {
        if (!goal_received_) return;

        // Check if we've exceeded the timeout
        double elapsed = (this->now() - goal_start_time_).seconds();
        if (elapsed > goal_timeout_sec) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Goal timeout after %.1f seconds! Stopping.", elapsed);
            emergency_stop();
            return;
        }

        // Get current robot pose from TF
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "TF lookup failed: %s", ex.what());
            return;
        }

        double x = tf.transform.translation.x;
        double y = tf.transform.translation.y;
        
        // Validate odometry (check for NaN or Inf)
        if (!std::isfinite(x) || !std::isfinite(y)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid odometry received! Stopping.");
            emergency_stop();
            return;
        }
        
        // Extract yaw angle from quaternion
        tf2::Quaternion q;
        tf2::fromMsg(tf.transform.rotation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Calculate distance and angle to goal
        double dx = goal_x_ - x;
        double dy = goal_y_ - y;
        double distance = std::sqrt(dx*dx + dy*dy);
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = target_yaw - yaw;
        
        // Normalize angle to [-π, π]
        while (yaw_error > M_PI) yaw_error -= 2*M_PI;
        while (yaw_error < -M_PI) yaw_error += 2*M_PI;

        auto cmd = rover_messages::msg::DriveMoveCmd();

        // Check if goal is reached
        if (distance < goal_tolerance) {
            cmd.x_cmd = 0.0;
            cmd.y_cmd = 0.0;
            cmd.yaw_cmd = 0.0;
            move_cmd_pub_->publish(cmd);
            goal_received_ = false;
            RCLCPP_INFO(this->get_logger(), "Goal reached! Final distance: %.3fm", distance);
            return;
        }

        // Normalize distance and rotation to [-1, 1]
        double normalized_distance = std::clamp(distance / max_distance, 0.0, 1.0);
        double normalized_yaw_error = std::clamp(yaw_error / M_PI, -1.0, 1.0);

        // Decide between turning in place vs driving
        if (std::abs(yaw_error) > turn_threshold) {
            // TURN IN PLACE MODE
            // Switch to NPT mode for rotation
            send_drive_mode(DriveMode::NPT);
            
            cmd.x_cmd = 0.0;  // No forward motion
            cmd.y_cmd = 0.0;  // No lateral motion
            cmd.yaw_cmd = normalized_yaw_error * max_angular_vel;
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Turning: yaw_error=%.2f° (%.3f rad), normalized=%.3f", 
                               yaw_error * 180.0 / M_PI, yaw_error, normalized_yaw_error);
        } else {
            // DRIVE FORWARD MODE
            // Switch to LINEAR mode for driving
            send_drive_mode(DriveMode::LINEAR);
            
            // Proportional control for velocity (using normalized values)
            cmd.x_cmd = normalized_distance * max_linear_vel;
            cmd.y_cmd = 0.0;
            cmd.yaw_cmd = normalized_yaw_error * max_angular_vel;
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Driving: dist=%.2fm (norm=%.3f), vel=%.2fm/s, time=%.1fs", 
                               distance, normalized_distance, cmd.x_cmd, elapsed);
        }

        move_cmd_pub_->publish(cmd);
    }

    // TF for odometry
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<rover_messages::msg::AutonDrive>::SharedPtr drive_pos_;
    
    // Publishers
    rclcpp::Publisher<rover_messages::msg::DriveMoveCmd>::SharedPtr move_cmd_pub_;
    rclcpp::Publisher<rover_messages::msg::DriveOpCmd>::SharedPtr op_cmd_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State
    bool goal_received_;
    double goal_x_, goal_y_;
    rclcpp::Time goal_start_time_;
    DriveMode current_mode_;  // Track current drive mode
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalDriver>());
    rclcpp::shutdown();
    return 0;
}

// ros2 topic pub /drive_pos rover_messages/msg/AutonDrive "{x_cmd: 0.0, y_cmd: 0.0, stop: false}" --once