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
#include "rover_messages/msg/manager_cmd.hpp"
#include "rover_messages/Enums.hpp"

class GoalDriver : public rclcpp::Node {    
public:
    GoalDriver() : Node("goal_driver_node"), goal_received_(false), current_mode_(DriveMode::NPT) {
        
        // --- DECLARE PARAMETERS ---
        this->declare_parameter("goal_timeout_sec", 45.0);
        this->declare_parameter("goal_tolerance", 0.15);
        this->declare_parameter("turn_threshold", 0.4);
        this->declare_parameter("max_distance", 2.5);
        
        // Fetch initial values
        update_params();
        
        // TF setup
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // --- SUBSCRIPTIONS ---
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&GoalDriver::goal_callback, this, std::placeholders::_1));
        
        drive_pos_ = this->create_subscription<rover_messages::msg::AutonDrive>(
            "/drive_pos", 10, std::bind(&GoalDriver::pos_callback, this, std::placeholders::_1));
        
        manager_sub_ = this->create_subscription<rover_messages::msg::ManagerCmd>(
            "rover/manager/cmd", 10, std::bind(&GoalDriver::manager_callback, this, std::placeholders::_1));
        
        // --- PUBLISHERS ---
        move_cmd_pub_ = this->create_publisher<rover_messages::msg::DriveMoveCmd>("rover/drive/move_cmd", 10);
        op_cmd_pub_ = this->create_publisher<rover_messages::msg::DriveOpCmd>("rover/drive/op_cmd", 10);
        
        // Control loop timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&GoalDriver::control_loop, this));
        
        send_drive_mode(DriveMode::NPT);
        RCLCPP_INFO(this->get_logger(), "Goal driver initialized");
    }

private:
    void update_params() {
        goal_timeout_sec_ = this->get_parameter("goal_timeout_sec").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        turn_threshold_ = this->get_parameter("turn_threshold").as_double();
        max_distance_ = this->get_parameter("max_distance").as_double();
    }
    
    void manager_callback(const rover_messages::msg::ManagerCmd::SharedPtr msg) {
        if (msg->shutdown) {
            if (!system_stopped_) {
                RCLCPP_WARN(this->get_logger(), "MANAGER STOP: Halting all motion");
                emergency_stop();
            }
            system_stopped_ = true;
        } else {
            if (system_stopped_) {
                RCLCPP_INFO(this->get_logger(), "MANAGER RESUME: System ready");
            }
            system_stopped_ = false;
        }
    }
    
    void send_drive_mode(DriveMode mode) {
        if (mode == current_mode_) return;  // Avoid spam
        
        auto op_cmd = rover_messages::msg::DriveOpCmd();
        op_cmd.drive_mode = static_cast<uint8_t>(mode);
        op_cmd.drive_limit = static_cast<uint8_t>(DriveLimits::NO_UPDATE);
        op_cmd_pub_->publish(op_cmd);
        
        current_mode_ = mode;
        
        const char* mode_name = (mode == DriveMode::NPT) ? "NPT" :
                               (mode == DriveMode::LINEAR) ? "LINEAR" :
                               (mode == DriveMode::ACKERMANN) ? "ACKERMANN" :
                               (mode == DriveMode::CRABERMANN) ? "CRABERMANN" : "UNKNOWN";
        
        RCLCPP_INFO(this->get_logger(), "Drive mode: %s", mode_name);
    }
    
    void emergency_stop() {
        auto cmd = rover_messages::msg::DriveMoveCmd();
        cmd.x_cmd = 0.0;
        cmd.y_cmd = 0.0;
        cmd.yaw_cmd = 0.0;
        move_cmd_pub_->publish(cmd);
        goal_received_ = false;
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP");
    }
    
    void pos_callback(const rover_messages::msg::AutonDrive::SharedPtr msg) {
        if (msg->stop) {
            RCLCPP_WARN(this->get_logger(), "Stop command via /drive_pos");
            emergency_stop();
            return;
        }
        
        if (!std::isfinite(msg->x_cmd) || !std::isfinite(msg->y_cmd)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid position (NaN/Inf)! Stopping.");
            emergency_stop();
            return;
        }
        
        goal_x_ = msg->x_cmd;
        goal_y_ = msg->y_cmd;
        goal_received_ = true;
        goal_start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "New goal from /drive_pos: (%.2f, %.2f)", goal_x_, goal_y_);
    }
    
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!std::isfinite(msg->pose.position.x) || 
            !std::isfinite(msg->pose.position.y) ||
            !std::isfinite(msg->pose.orientation.z) ||
            !std::isfinite(msg->pose.orientation.w)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid goal pose (NaN/Inf)! Stopping.");
            emergency_stop();
            return;
        }
        
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        goal_received_ = true;
        goal_start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "New goal from /goal_pose: (%.2f, %.2f)", goal_x_, goal_y_);
    }
    
    void control_loop() {
        if (!goal_received_ || system_stopped_) return;
        
        update_params();
        
        // Check timeout
        double elapsed = (this->now() - goal_start_time_).seconds();
        if (elapsed > goal_timeout_sec_) {
            RCLCPP_ERROR(this->get_logger(), "Goal timeout (%.1fs)! Stopping.", elapsed);
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
        
        if (!std::isfinite(x) || !std::isfinite(y)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid odometry (NaN/Inf)! Stopping.");
            emergency_stop();
            return;
        }
        
        // Extract yaw from quaternion
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
        
        // Check if goal reached
        if (distance < goal_tolerance_) {
            cmd.x_cmd = 0.0;
            cmd.y_cmd = 0.0;
            cmd.yaw_cmd = 0.0;
            move_cmd_pub_->publish(cmd);
            goal_received_ = false;
            RCLCPP_INFO(this->get_logger(), "Goal reached! Distance: %.3fm", distance);
            return;
        }
        
        // Normalize to [0, 1] for speed and [-1, 1] for rotation
        double normalized_distance = std::clamp(distance / max_distance_, 0.0, 1.0);
        double normalized_yaw_error = std::clamp(yaw_error / M_PI, -1.0, 1.0);
        
        // Decide: turn in place vs drive
        if (std::abs(yaw_error) > turn_threshold_) {
            // TURN IN PLACE MODE
            send_drive_mode(DriveMode::NPT);
            
            cmd.x_cmd = 0.0;                    
            cmd.y_cmd = 0.0;                    // no lateral
            cmd.yaw_cmd = normalized_yaw_error; // [-1, 1] - normalized rotation
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Turning: yaw_error=%.1f° (cmd=%.2f)", 
                               yaw_error * 180.0 / M_PI, cmd.yaw_cmd);
        } else {
            // DRIVE FORWARD MODE
            send_drive_mode(DriveMode::LINEAR);
            
            cmd.x_cmd = normalized_distance;   
            cmd.y_cmd = 0.0;                    // no lateral
            cmd.yaw_cmd = normalized_yaw_error; // [-1, 1]
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Driving: dist=%.2fm, speed_cmd=%.2f, yaw_cmd=%.2f, t=%.1fs", 
                               distance, cmd.x_cmd, cmd.yaw_cmd, elapsed);
        }
        
        move_cmd_pub_->publish(cmd);
    }
    
    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<rover_messages::msg::AutonDrive>::SharedPtr drive_pos_;
    rclcpp::Subscription<rover_messages::msg::ManagerCmd>::SharedPtr manager_sub_;
    
    // Publishers
    rclcpp::Publisher<rover_messages::msg::DriveMoveCmd>::SharedPtr move_cmd_pub_;
    rclcpp::Publisher<rover_messages::msg::DriveOpCmd>::SharedPtr op_cmd_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State
    bool system_stopped_ = false;
    bool goal_received_;
    double goal_x_, goal_y_;
    rclcpp::Time goal_start_time_;
    DriveMode current_mode_;
    
    // Parameters (no max_linear_vel or max_angular_vel - not needed!)
    double goal_timeout_sec_;
    double goal_tolerance_;
    double turn_threshold_;
    double max_distance_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalDriver>());
    rclcpp::shutdown();
    return 0;
}

// USAGE:
// ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}" --once
// CUSTOM MSG
// ros2 topic pub /drive_pos rover_messages/msg/AutonDrive "{x_cmd: 3.0, y_cmd: -1.5, stop: false}" --once