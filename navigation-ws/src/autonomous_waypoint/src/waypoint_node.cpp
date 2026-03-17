#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rtabmap_msgs/msg/info.hpp>
#include <cmath>
#include <algorithm>
#include "rover_messages/msg/drive_move_cmd.hpp"
#include "rover_messages/msg/drive_op_cmd.hpp"
#include "rover_messages/msg/auton_drive.hpp"
#include "rover_messages/msg/manager_cmd.hpp"
#include "rover_messages/Enums.hpp"

// ---------------------------------------------------------------------------
// Drive state machine
//
//   IDLE       → no active goal
//   TURNING    → rotating in place to face goal (NPT mode)
//                exits when |yaw_error| < turn_exit_threshold_
//   DRIVING    → moving toward goal (LINEAR mode)
//                continuously corrects heading while driving
//                re-enters TURNING if |yaw_error| > turn_enter_threshold_
//   RECOVERING → RTABMap tracking quality degraded, holding still
//                exits when tracking recovers, re-enters TURNING
// ---------------------------------------------------------------------------
enum class DriveState { IDLE, TURNING, DRIVING, RECOVERING };

class GoalDriver : public rclcpp::Node {
public:
    GoalDriver()
    : Node("waypoint_node"),
      state_(DriveState::IDLE),
      current_mode_(DriveMode::NPT)
    {
        // --- PARAMETERS ---
        this->declare_parameter("goal_timeout_sec",      45.0);
        this->declare_parameter("goal_tolerance",         0.15);

        // Hysteresis: must exceed ENTER to trigger re-turn,
        // must drop below EXIT to exit TURNING and start DRIVING.
        // EXIT < ENTER prevents mode oscillation.
        this->declare_parameter("turn_enter_threshold",   0.35);  // ~20 deg
        this->declare_parameter("turn_exit_threshold",    0.15);  // ~8.5 deg

        this->declare_parameter("max_distance",           2.5);
        this->declare_parameter("drive_speed",            0.5);   // fixed [0,1]

        // RTABMap tracking quality thresholds
        this->declare_parameter("min_visual_words",       20);
        this->declare_parameter("max_odom_variance",      0.1);

        // How long (s) to wait in RECOVERING before aborting the goal
        this->declare_parameter("recovery_timeout_sec",   5.0);

        update_params();

        // TF
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // --- SUBSCRIPTIONS ---
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&GoalDriver::goal_callback, this, std::placeholders::_1));

        drive_pos_ = this->create_subscription<rover_messages::msg::AutonDrive>(
            "/drive_pos", 10,
            std::bind(&GoalDriver::pos_callback, this, std::placeholders::_1));

        manager_sub_ = this->create_subscription<rover_messages::msg::ManagerCmd>(
            "rover/manager/cmd", 10,
            std::bind(&GoalDriver::manager_callback, this, std::placeholders::_1));

        // RTABMap info — tracks visual feature count, fast movement flag,
        // and odometry variance to detect when positioning is unreliable.
        info_sub_ = this->create_subscription<rtabmap_msgs::msg::Info>(
            "/info", 10,
            std::bind(&GoalDriver::info_callback, this, std::placeholders::_1));

        // --- PUBLISHERS ---
        move_cmd_pub_ = this->create_publisher<rover_messages::msg::DriveMoveCmd>(
            "rover/drive/drive_move_cmd", 10);
        op_cmd_pub_ = this->create_publisher<rover_messages::msg::DriveOpCmd>(
            "rover/drive/drive_op_cmd", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GoalDriver::control_loop, this));

        send_drive_mode(DriveMode::NPT);
        RCLCPP_INFO(this->get_logger(), "Waypoint driver initialized");
    }

private:
    void update_params() {
        goal_timeout_sec_     = this->get_parameter("goal_timeout_sec").as_double();
        goal_tolerance_       = this->get_parameter("goal_tolerance").as_double();
        turn_enter_threshold_ = this->get_parameter("turn_enter_threshold").as_double();
        turn_exit_threshold_  = this->get_parameter("turn_exit_threshold").as_double();
        max_distance_         = this->get_parameter("max_distance").as_double();
        drive_speed_          = this->get_parameter("drive_speed").as_double();
        min_visual_words_     = this->get_parameter("min_visual_words").as_int();
        max_odom_variance_    = this->get_parameter("max_odom_variance").as_double();
        recovery_timeout_sec_ = this->get_parameter("recovery_timeout_sec").as_double();
    }

    // -----------------------------------------------------------------------
    // RTABMap info callback — tracking quality monitor
    // Monitors three signals from /info stats:
    //   Keypoint/Current_frame/words  — 0 means no visual features, totally lost
    //   Memory/Fast_movement/         — 1 means motion too fast for tracker
    //   Memory/Odometry_variance_lin  — high value means odometry is uncertain
    // -----------------------------------------------------------------------
    void info_callback(const rtabmap_msgs::msg::Info::SharedPtr msg) {
        int    visual_words  = -1;    // -1 = key not yet seen
        bool   fast_movement = false;
        double odom_var_lin  = 0.0;

        for (size_t i = 0; i < msg->stats_keys.size(); ++i) {
            const auto& key = msg->stats_keys[i];
            const float val = msg->stats_values[i];

            if (key == "Keypoint/Current_frame/words")
                visual_words = static_cast<int>(val);
            else if (key == "Memory/Fast_movement/")
                fast_movement = (val > 0.0f);
            else if (key == "Memory/Odometry_variance_lin")
                odom_var_lin = static_cast<double>(val);
        }

        bool new_tracking_ok = true;

        if (visual_words >= 0 && visual_words < min_visual_words_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "RTABMap: low visual features (%d < %d) — tracking unreliable",
                visual_words, min_visual_words_);
            new_tracking_ok = false;
        }
        if (fast_movement) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "RTABMap: fast movement flag set — tracking unreliable");
            new_tracking_ok = false;
        }
        if (odom_var_lin > max_odom_variance_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "RTABMap: high odom variance (%.4f > %.4f) — tracking unreliable",
                odom_var_lin, max_odom_variance_);
            new_tracking_ok = false;
        }

        tracking_ok_ = new_tracking_ok;
    }

    // -----------------------------------------------------------------------
    // Callbacks
    // -----------------------------------------------------------------------
    void manager_callback(const rover_messages::msg::ManagerCmd::SharedPtr msg) {
        if (msg->shutdown) {
            if (!system_stopped_) {
                RCLCPP_WARN(this->get_logger(), "MANAGER STOP");
                emergency_stop();
            }
            system_stopped_ = true;
        } else {
            if (system_stopped_)
                RCLCPP_INFO(this->get_logger(), "MANAGER RESUME");
            system_stopped_ = false;
        }
    }

    void pos_callback(const rover_messages::msg::AutonDrive::SharedPtr msg) {
        if (msg->stop) { emergency_stop(); return; }
        if (!std::isfinite(msg->x_cmd) || !std::isfinite(msg->y_cmd)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid goal (NaN/Inf) — ignoring");
            return;
        }
        set_goal(msg->x_cmd, msg->y_cmd, "/drive_pos");
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!std::isfinite(msg->pose.position.x) ||
            !std::isfinite(msg->pose.position.y)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid goal (NaN/Inf) — ignoring");
            return;
        }
        set_goal(msg->pose.position.x, msg->pose.position.y, "/goal_pose");
    }

    void set_goal(double x, double y, const char* source) {
        goal_x_          = x;
        goal_y_          = y;
        goal_start_time_ = this->now();
        transition_to(DriveState::TURNING);
        RCLCPP_INFO(this->get_logger(),
            "New goal from %s: (%.2f, %.2f) — entering TURNING", source, x, y);
    }

    // -----------------------------------------------------------------------
    // State transitions
    // -----------------------------------------------------------------------
    void transition_to(DriveState new_state) {
        if (state_ == new_state) return;
        const char* names[] = {"IDLE", "TURNING", "DRIVING", "RECOVERING"};
        RCLCPP_INFO(this->get_logger(), "State: %s → %s",
            names[static_cast<int>(state_)],
            names[static_cast<int>(new_state)]);
        state_ = new_state;
        if (new_state == DriveState::RECOVERING)
            recovery_start_time_ = this->now();
    }

    // -----------------------------------------------------------------------
    // Main control loop (10 Hz)
    // -----------------------------------------------------------------------
    void control_loop() {
        if (system_stopped_) return;
        if (state_ == DriveState::IDLE) return;

        update_params();

        double elapsed = (this->now() - goal_start_time_).seconds();
        if (elapsed > goal_timeout_sec_) {
            RCLCPP_ERROR(this->get_logger(), "Goal timeout (%.1fs) — aborting", elapsed);
            emergency_stop();
            return;
        }

        // Tracking quality gate
        if (!tracking_ok_ && state_ != DriveState::RECOVERING) {
            RCLCPP_WARN(this->get_logger(),
                "Tracking degraded — RECOVERING, holding position");
            publish_stop();
            transition_to(DriveState::RECOVERING);
            return;
        }

        // Get pose from TF
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform(
                "odom", "zed_camera_link", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "TF lookup failed: %s", ex.what());
            return;
        }

        double x = tf.transform.translation.x;
        double y = tf.transform.translation.y;
        if (!std::isfinite(x) || !std::isfinite(y)) {
            RCLCPP_ERROR(this->get_logger(), "Non-finite odometry — stopping");
            emergency_stop();
            return;
        }

        tf2::Quaternion q;
        tf2::fromMsg(tf.transform.rotation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        double dx         = goal_x_ - x;
        double dy         = goal_y_ - y;
        double distance   = std::sqrt(dx*dx + dy*dy);
        double target_yaw = std::atan2(dy, dx);
        double yaw_error  = target_yaw - yaw;
        while (yaw_error >  M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

        switch (state_) {

        case DriveState::RECOVERING: {
            publish_stop();
            double rec_elapsed = (this->now() - recovery_start_time_).seconds();
            if (tracking_ok_) {
                RCLCPP_INFO(this->get_logger(),
                    "Tracking recovered after %.1fs — re-evaluating heading", rec_elapsed);
                transition_to(DriveState::TURNING);
            } else if (rec_elapsed > recovery_timeout_sec_) {
                RCLCPP_ERROR(this->get_logger(),
                    "Tracking did not recover in %.1fs — aborting goal",
                    recovery_timeout_sec_);
                emergency_stop();
            }
            return;
        }

        case DriveState::TURNING: {
            send_drive_mode(DriveMode::NPT);
            if (distance < goal_tolerance_) { goal_complete(distance); return; }

            double norm_yaw = std::clamp(yaw_error / M_PI, -1.0, 1.0);
            auto cmd = rover_messages::msg::DriveMoveCmd();
            cmd.x_cmd = 0.0; cmd.y_cmd = 0.0; cmd.yaw_cmd = norm_yaw;
            move_cmd_pub_->publish(cmd);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "TURNING: yaw_err=%.1f° cmd=%.2f",
                yaw_error * 180.0 / M_PI, norm_yaw);

            if (std::abs(yaw_error) < turn_exit_threshold_) {
                RCLCPP_INFO(this->get_logger(),
                    "Aligned (%.1f°) — entering DRIVING",
                    yaw_error * 180.0 / M_PI);
                transition_to(DriveState::DRIVING);
            }
            break;
        }

        case DriveState::DRIVING: {
            send_drive_mode(DriveMode::LINEAR);
            if (distance < goal_tolerance_) { goal_complete(distance); return; }

            // Stop and re-align if heading drift exceeds enter threshold
            if (std::abs(yaw_error) > turn_enter_threshold_) {
                RCLCPP_INFO(this->get_logger(),
                    "Heading drift (%.1f°) — re-entering TURNING",
                    yaw_error * 180.0 / M_PI);
                publish_stop();
                transition_to(DriveState::TURNING);
                return;
            }

            double norm_yaw = std::clamp(yaw_error / M_PI, -1.0, 1.0);
            auto cmd = rover_messages::msg::DriveMoveCmd();
            cmd.x_cmd = drive_speed_; cmd.y_cmd = 0.0; cmd.yaw_cmd = norm_yaw;
            move_cmd_pub_->publish(cmd);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "DRIVING: dist=%.2fm yaw_err=%.1f° speed=%.2f t=%.1fs",
                distance, yaw_error * 180.0 / M_PI, drive_speed_, elapsed);
            break;
        }

        default: break;
        }
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------
    void goal_complete(double distance) {
        publish_stop();
        transition_to(DriveState::IDLE);
        RCLCPP_INFO(this->get_logger(), "Goal reached! Final distance: %.3fm", distance);
    }

    void publish_stop() {
        auto cmd = rover_messages::msg::DriveMoveCmd();
        cmd.x_cmd = 0.0; cmd.y_cmd = 0.0; cmd.yaw_cmd = 0.0;
        move_cmd_pub_->publish(cmd);
    }

    void emergency_stop() {
        publish_stop();
        transition_to(DriveState::IDLE);
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP");
    }

    void send_drive_mode(DriveMode mode) {
        if (mode == current_mode_) return;
        auto op_cmd = rover_messages::msg::DriveOpCmd();
        op_cmd.drive_mode  = static_cast<uint8_t>(mode);
        op_cmd.drive_limit = static_cast<uint8_t>(DriveLimits::NO_UPDATE);
        op_cmd_pub_->publish(op_cmd);
        current_mode_ = mode;
        const char* name =
            (mode == DriveMode::NPT)        ? "NPT"        :
            (mode == DriveMode::LINEAR)     ? "LINEAR"     :
            (mode == DriveMode::ACKERMANN)  ? "ACKERMANN"  :
            (mode == DriveMode::CRABERMANN) ? "CRABERMANN" : "UNKNOWN";
        RCLCPP_INFO(this->get_logger(), "Drive mode → %s", name);
    }

    // -----------------------------------------------------------------------
    // Members
    // -----------------------------------------------------------------------
    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<rover_messages::msg::AutonDrive>::SharedPtr drive_pos_;
    rclcpp::Subscription<rover_messages::msg::ManagerCmd>::SharedPtr manager_sub_;
    rclcpp::Subscription<rtabmap_msgs::msg::Info>::SharedPtr         info_sub_;

    rclcpp::Publisher<rover_messages::msg::DriveMoveCmd>::SharedPtr  move_cmd_pub_;
    rclcpp::Publisher<rover_messages::msg::DriveOpCmd>::SharedPtr    op_cmd_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    DriveState state_;
    DriveMode  current_mode_;

    bool         system_stopped_  = false;
    bool         tracking_ok_     = true;  // permissive until first /info msg

    double       goal_x_          = 0.0;
    double       goal_y_          = 0.0;
    rclcpp::Time goal_start_time_;
    rclcpp::Time recovery_start_time_;

    double goal_timeout_sec_;
    double goal_tolerance_;
    double turn_enter_threshold_;
    double turn_exit_threshold_;
    double max_distance_;
    double drive_speed_;
    int    min_visual_words_;
    double max_odom_variance_;
    double recovery_timeout_sec_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalDriver>());
    rclcpp::shutdown();
    return 0;
}

// USAGE:
// ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
//   "{header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 1.0}}}" --once
//
// ros2 topic pub /drive_pos rover_messages/msg/AutonDrive \
//   "{x_cmd: 3.0, y_cmd: -1.5, stop: false}" --once