// waypoint_queue_node.cpp
//
// Accepts a list of (x, y) waypoints and feeds them one-by-one to the
// existing GoalDriver node via /drive_pos (rover_messages/AutonDrive).
//
// The queue manager watches GoalDriver's state indirectly: it listens to
// /drive_pos_ack (see below) OR it monitors the rover's odometry via TF
// to detect when the rover has settled within tolerance of the current
// waypoint, then advances to the next one.
//
// ── Input interfaces ────────────────────────────────────────────────────────
//
//   LOAD queue (replaces any existing queue):
//     ros2 topic pub /waypoint_queue/load auton_waypoint/msg/WaypointList 
//       "{waypoints: [{x: 1.0, y: 0.0}, {x: 3.0, y: 2.0}, {x: 0.0, y: 0.0}]}"
//       --once
//
//   APPEND a single waypoint:
//     ros2 topic pub /waypoint_queue/append geometry_msgs/msg/Point 
//       "{x: 5.0, y: 1.5}" --once
//
//   START execution (begins sending waypoints):
//     ros2 topic pub /waypoint_queue/start std_msgs/msg/Empty "{}" --once
//
//   PAUSE execution (holds after current waypoint completes):
//     ros2 topic pub /waypoint_queue/pause std_msgs/msg/Empty "{}" --once
//
//   RESUME execution:
//     ros2 topic pub /waypoint_queue/resume std_msgs/msg/Empty "{}" --once
//
//   CANCEL execution (stops immediately, clears queue):
//     ros2 topic pub /waypoint_queue/cancel std_msgs/msg/Empty "{}" --once
//
// ── Output interfaces ───────────────────────────────────────────────────────
//
//   /drive_pos  rover_messages/msg/AutonDrive   — current active waypoint
//   /waypoint_queue/status  std_msgs/msg/String — human-readable state
//
// ── Advancement logic ───────────────────────────────────────────────────────
//
//   Every 100 ms the manager polls TF for the rover's current position.
//   When distance to the active waypoint drops below `goal_tolerance` for
//   `settle_count` consecutive cycles, the waypoint is marked complete and
//   the next one is dispatched.  This mirrors GoalDriver's own tolerance
//   check without requiring a dedicated acknowledgement topic.
//
// ── Parameters ──────────────────────────────────────────────────────────────
//
//   goal_tolerance   (double, default 0.15 m) — match GoalDriver's value
//   settle_count     (int,    default 3)       — consecutive cycles in tol.
//   waypoint_timeout (double, default 60.0 s)  — abort waypoint if exceeded
//   resend_interval  (double, default 2.0 s)   — re-publish goal periodically
//                                                 in case GoalDriver missed it

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <deque>
#include <string>
#include <sstream>

#include "rover_messages/msg/auton_drive.hpp"

// ---------------------------------------------------------------------------
// Minimal WaypointList message — define inline so we don't need a new .msg
// file if the package doesn't already have one.
// If your project already has auton_waypoint/msg/WaypointList, remove this
// struct and include that header instead; the rest of the code is unchanged.
// ---------------------------------------------------------------------------
struct Waypoint {
    double x;
    double y;
};

// ---------------------------------------------------------------------------
// Queue manager states
// ---------------------------------------------------------------------------
enum class QueueState {
    IDLE,       // no queue loaded / execution not started
    RUNNING,    // actively sending waypoints
    PAUSED,     // queue loaded, execution suspended
    COMPLETE,   // all waypoints reached
};

class WaypointQueueNode : public rclcpp::Node {
public:
    WaypointQueueNode()
    : Node("waypoint_queue_node"),
      queue_state_(QueueState::IDLE),
      active_index_(0),
      settle_counter_(0)
    {
        // ── Parameters ──────────────────────────────────────────────────
        this->declare_parameter("goal_tolerance",    0.15);
        this->declare_parameter("settle_count",      3);
        this->declare_parameter("waypoint_timeout",  60.0);
        this->declare_parameter("resend_interval",   2.0);
        update_params();

        // ── TF ──────────────────────────────────────────────────────────
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ── Subscribers ─────────────────────────────────────────────────

        // Load a full waypoint list (replaces existing queue)
        // Message type: std_msgs/String, encoded as "x1,y1;x2,y2;x3,y3"
        // Example:
        //   ros2 topic pub /waypoint_queue/load std_msgs/msg/String 
        //     "{data: '1.0,0.0;3.0,2.0;0.0,0.0'}" --once
        load_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/waypoint_queue/load", 10,
            std::bind(&WaypointQueueNode::load_callback, this, std::placeholders::_1));

        // Append a single waypoint: "x,y"
        // Example:
        //   ros2 topic pub /waypoint_queue/append std_msgs/msg/String 
        //     "{data: '5.0,1.5'}" --once
        append_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/waypoint_queue/append", 10,
            std::bind(&WaypointQueueNode::append_callback, this, std::placeholders::_1));

        start_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/waypoint_queue/start", 10,
            std::bind(&WaypointQueueNode::start_callback, this, std::placeholders::_1));

        pause_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/waypoint_queue/pause", 10,
            std::bind(&WaypointQueueNode::pause_callback, this, std::placeholders::_1));

        resume_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/waypoint_queue/resume", 10,
            std::bind(&WaypointQueueNode::resume_callback, this, std::placeholders::_1));

        cancel_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/waypoint_queue/cancel", 10,
            std::bind(&WaypointQueueNode::cancel_callback, this, std::placeholders::_1));

        // ── Publishers ──────────────────────────────────────────────────
        drive_pub_ = this->create_publisher<rover_messages::msg::AutonDrive>(
            "/drive_pos", 10);

        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/waypoint_queue/status", 10);

        // ── Control loop (10 Hz) ────────────────────────────────────────
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&WaypointQueueNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Waypoint queue node ready.");
        RCLCPP_INFO(this->get_logger(),
            "Load queue : ros2 topic pub /waypoint_queue/load "
            "std_msgs/msg/String \"{data: '1.0,0.0;3.0,2.0'}\" --once");
        RCLCPP_INFO(this->get_logger(),
            "Then start : ros2 topic pub /waypoint_queue/start "
            "std_msgs/msg/Empty \"{}\" --once");
    }

private:
    // ── Parameter refresh ───────────────────────────────────────────────────
    void update_params() {
        goal_tolerance_   = this->get_parameter("goal_tolerance").as_double();
        settle_count_     = this->get_parameter("settle_count").as_int();
        waypoint_timeout_ = this->get_parameter("waypoint_timeout").as_double();
        resend_interval_  = this->get_parameter("resend_interval").as_double();
    }

    // ── Parsing helpers ─────────────────────────────────────────────────────
    // Parse "x,y" into a Waypoint.  Returns false on failure.
    bool parse_waypoint(const std::string& token, Waypoint& wp) {
        auto comma = token.find(',');
        if (comma == std::string::npos) return false;
        try {
            wp.x = std::stod(token.substr(0, comma));
            wp.y = std::stod(token.substr(comma + 1));
            return true;
        } catch (...) {
            return false;
        }
    }

    // Parse "x1,y1;x2,y2;..." into a vector of Waypoints.
    std::vector<Waypoint> parse_list(const std::string& data) {
        std::vector<Waypoint> result;
        std::stringstream ss(data);
        std::string token;
        while (std::getline(ss, token, ';')) {
            if (token.empty()) continue;
            Waypoint wp;
            if (parse_waypoint(token, wp))
                result.push_back(wp);
            else
                RCLCPP_WARN(this->get_logger(),
                    "Could not parse waypoint token: '%s'", token.c_str());
        }
        return result;
    }

    // ── Subscription callbacks ───────────────────────────────────────────────
    void load_callback(const std_msgs::msg::String::SharedPtr msg) {
        auto wps = parse_list(msg->data);
        if (wps.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                "load: no valid waypoints parsed from '%s'", msg->data.c_str());
            return;
        }
        queue_.clear();
        for (auto& wp : wps) queue_.push_back(wp);
        active_index_   = 0;
        settle_counter_ = 0;
        queue_state_    = QueueState::IDLE;   // require explicit start
        RCLCPP_INFO(this->get_logger(),
            "Queue loaded: %zu waypoints. Send /waypoint_queue/start to begin.",
            queue_.size());
        log_queue();
        publish_status();
    }

    void append_callback(const std_msgs::msg::String::SharedPtr msg) {
        Waypoint wp;
        if (!parse_waypoint(msg->data, wp)) {
            RCLCPP_ERROR(this->get_logger(),
                "append: could not parse '%s' as 'x,y'", msg->data.c_str());
            return;
        }
        queue_.push_back(wp);
        RCLCPP_INFO(this->get_logger(),
            "Appended waypoint (%.2f, %.2f). Queue size: %zu",
            wp.x, wp.y, queue_.size());
        publish_status();
    }

    void start_callback(const std_msgs::msg::Empty::SharedPtr) {
        if (queue_.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "start: queue is empty — load waypoints first.");
            return;
        }
        if (queue_state_ == QueueState::RUNNING) {
            RCLCPP_WARN(this->get_logger(), "start: already running.");
            return;
        }
        active_index_        = 0;
        settle_counter_      = 0;
        waypoint_start_time_ = this->now();
        last_send_time_      = rclcpp::Time(0, 0, RCL_ROS_TIME);
        queue_state_         = QueueState::RUNNING;
        RCLCPP_INFO(this->get_logger(),
            "Queue started — %zu waypoints to execute.", queue_.size());
        dispatch_current();
        publish_status();
    }

    void pause_callback(const std_msgs::msg::Empty::SharedPtr) {
        if (queue_state_ != QueueState::RUNNING) {
            RCLCPP_WARN(this->get_logger(), "pause: not currently running.");
            return;
        }
        queue_state_ = QueueState::PAUSED;
        RCLCPP_INFO(this->get_logger(),
            "Queue PAUSED at waypoint %zu/%zu.",
            active_index_ + 1, queue_.size());
        publish_status();
    }

    void resume_callback(const std_msgs::msg::Empty::SharedPtr) {
        if (queue_state_ != QueueState::PAUSED) {
            RCLCPP_WARN(this->get_logger(), "resume: not paused.");
            return;
        }
        queue_state_         = QueueState::RUNNING;
        waypoint_start_time_ = this->now();   // reset timeout on resume
        last_send_time_      = rclcpp::Time(0, 0, RCL_ROS_TIME);
        RCLCPP_INFO(this->get_logger(), "Queue RESUMED.");
        dispatch_current();
        publish_status();
    }

    void cancel_callback(const std_msgs::msg::Empty::SharedPtr) {
        send_stop();
        queue_.clear();
        active_index_   = 0;
        settle_counter_ = 0;
        queue_state_    = QueueState::IDLE;
        RCLCPP_WARN(this->get_logger(), "Queue CANCELLED — queue cleared.");
        publish_status();
    }

    // ── Control loop ────────────────────────────────────────────────────────
    void control_loop() {
        update_params();
        publish_status();

        if (queue_state_ != QueueState::RUNNING) return;
        if (active_index_ >= queue_.size()) {
            // All waypoints done
            queue_state_ = QueueState::COMPLETE;
            RCLCPP_INFO(this->get_logger(),
                "All %zu waypoints complete!", queue_.size());
            publish_status();
            return;
        }

        // ── Timeout check ──────────────────────────────────────────────
        double elapsed = (this->now() - waypoint_start_time_).seconds();
        if (elapsed > waypoint_timeout_) {
            RCLCPP_ERROR(this->get_logger(),
                "Waypoint %zu/%zu timed out after %.1fs — skipping.",
                active_index_ + 1, queue_.size(), elapsed);
            advance_to_next();
            return;
        }

        // ── Periodic re-send ───────────────────────────────────────────
        // GoalDriver accepts goals via /drive_pos but has no persistent
        // goal memory — if it restarts or misses the message we resend.
        double since_send =
            (last_send_time_.nanoseconds() == 0)
            ? resend_interval_ + 1.0
            : (this->now() - last_send_time_).seconds();
        if (since_send >= resend_interval_)
            dispatch_current();

        // ── Pose from TF ───────────────────────────────────────────────
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform(
                "odom", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF lookup failed: %s", ex.what());
            return;
        }

        double rx = tf.transform.translation.x;
        double ry = tf.transform.translation.y;
        const Waypoint& wp = queue_[active_index_];
        double dx  = wp.x - rx;
        double dy  = wp.y - ry;
        double dist = std::sqrt(dx*dx + dy*dy);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "WP %zu/%zu → (%.2f, %.2f)  dist=%.3fm  t=%.1fs",
            active_index_ + 1, queue_.size(), wp.x, wp.y, dist, elapsed);

        // ── Settle check ───────────────────────────────────────────────
        if (dist < goal_tolerance_) {
            settle_counter_++;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                "Within tolerance (%.3fm), settle %d/%d",
                dist, settle_counter_, settle_count_);
            if (settle_counter_ >= settle_count_) {
                RCLCPP_INFO(this->get_logger(),
                    "Waypoint %zu/%zu reached (%.3fm). Advancing.",
                    active_index_ + 1, queue_.size(), dist);
                advance_to_next();
            }
        } else {
            settle_counter_ = 0;
        }
    }

    // ── Helpers ─────────────────────────────────────────────────────────────
    void dispatch_current() {
        if (active_index_ >= queue_.size()) return;
        const Waypoint& wp = queue_[active_index_];
        auto msg = rover_messages::msg::AutonDrive();
        msg.x_cmd = wp.x;
        msg.y_cmd = wp.y;
        msg.stop  = false;
        drive_pub_->publish(msg);
        last_send_time_ = this->now();
        RCLCPP_INFO(this->get_logger(),
            "Dispatched WP %zu/%zu → (%.2f, %.2f)",
            active_index_ + 1, queue_.size(), wp.x, wp.y);
    }

    void advance_to_next() {
        active_index_++;
        settle_counter_      = 0;
        waypoint_start_time_ = this->now();
        last_send_time_      = rclcpp::Time(0, 0, RCL_ROS_TIME);
        if (active_index_ < queue_.size()) {
            RCLCPP_INFO(this->get_logger(),
                "Advancing to waypoint %zu/%zu",
                active_index_ + 1, queue_.size());
            dispatch_current();
        }
        // completion is handled at top of control_loop on next tick
    }

    void send_stop() {
        auto msg = rover_messages::msg::AutonDrive();
        msg.x_cmd = 0.0; msg.y_cmd = 0.0; msg.stop = true;
        drive_pub_->publish(msg);
    }

    void publish_status() {
        const char* state_str =
            queue_state_ == QueueState::IDLE     ? "IDLE"     :
            queue_state_ == QueueState::RUNNING  ? "RUNNING"  :
            queue_state_ == QueueState::PAUSED   ? "PAUSED"   :
            queue_state_ == QueueState::COMPLETE ? "COMPLETE" : "UNKNOWN";

        std::ostringstream ss;
        ss << state_str
           << " | wp " << (active_index_ + 1)
           << "/" << queue_.size();
        if (!queue_.empty() && active_index_ < queue_.size())
            ss << " | next=(%.2f,%.2f)"
               << queue_[active_index_].x << ","
               << queue_[active_index_].y;

        auto msg = std_msgs::msg::String();
        msg.data = ss.str();
        status_pub_->publish(msg);
    }

    void log_queue() {
        for (size_t i = 0; i < queue_.size(); ++i)
            RCLCPP_INFO(this->get_logger(),
                "  [%zu] (%.2f, %.2f)", i + 1, queue_[i].x, queue_[i].y);
    }

    // ── Members ─────────────────────────────────────────────────────────────
    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr load_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr append_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr  start_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr  pause_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr  resume_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr  cancel_sub_;

    rclcpp::Publisher<rover_messages::msg::AutonDrive>::SharedPtr drive_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           status_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Waypoint> queue_;
    QueueState            queue_state_;
    size_t                active_index_;
    int                   settle_counter_;

    rclcpp::Time waypoint_start_time_;
    rclcpp::Time last_send_time_;

    // params
    double goal_tolerance_;
    int    settle_count_;
    double waypoint_timeout_;
    double resend_interval_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointQueueNode>());
    rclcpp::shutdown();
    return 0;
}

// =============================================================================
// QUICK REFERENCE
// =============================================================================
//
// Load 3 waypoints and execute:
//
//   ros2 topic pub /waypoint_queue/load std_msgs/msg/String 
//     "{data: '1.0,0.0;3.0,2.0;0.0,0.0'}" --once
//
//   ros2 topic pub /waypoint_queue/start std_msgs/msg/Empty "{}" --once
//
// Append one more waypoint while running:
//
//   ros2 topic pub /waypoint_queue/append std_msgs/msg/String 
//     "{data: '5.0,1.5'}" --once
//
// Pause / resume:
//
//   ros2 topic pub /waypoint_queue/pause  std_msgs/msg/Empty "{}" --once
//   ros2 topic pub /waypoint_queue/resume std_msgs/msg/Empty "{}" --once
//
// Cancel and clear:
//
//   ros2 topic pub /waypoint_queue/cancel std_msgs/msg/Empty "{}" --once
//
// Monitor progress:
//
//   ros2 topic echo /waypoint_queue/status
// =============================================================================