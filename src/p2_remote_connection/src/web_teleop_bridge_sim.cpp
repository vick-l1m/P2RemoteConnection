#include <chrono>
#include <string>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"   // NEW

using namespace std::chrono_literals;

class WebTeleopBridgeSim : public rclcpp::Node {
public:
  WebTeleopBridgeSim()
  : Node("web_teleop_bridge_sim")
  {
    // Parameters so you can choose which simulated robot to drive
    this->declare_parameter<int>("robot_index", 0);
    this->declare_parameter<double>("timeout_s", 0.25);

    robot_index_ = this->get_parameter("robot_index").as_int();
    timeout_s_ = this->get_parameter("timeout_s").as_double();

    // Topics (SIM-UNIQUE - keep)
    web_teleop_topic_ = "/web_teleop";
    web_action_topic_ = "/web_action";
    sim_cmd_vel_topic_ = "/robot" + std::to_string(robot_index_) + "/cmd_vel";

    teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      web_teleop_topic_, 10,
      std::bind(&WebTeleopBridgeSim::teleopCb, this, std::placeholders::_1));

    action_sub_ = this->create_subscription<std_msgs::msg::String>(
      web_action_topic_, 10,
      std::bind(&WebTeleopBridgeSim::actionCb, this, std::placeholders::_1));

    // NEW: stairs detection subscriber (same topic name as real robot)
    stairs_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/stairs_detected", 10,
      std::bind(&WebTeleopBridgeSim::stairsCb, this, std::placeholders::_1));

    // Publish to sim cmd_vel (SIM-UNIQUE)
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(sim_cmd_vel_topic_, 10);

    // 50 ms ~= 20 Hz output stream
    timer_ = this->create_wall_timer(50ms, std::bind(&WebTeleopBridgeSim::tick, this));

    last_rx_time_ = this->now();
    last_stairs_true_ = this->now() - rclcpp::Duration::from_seconds(1000.0);

    RCLCPP_INFO(this->get_logger(),
      "WebTeleopBridgeSim started. Subscribing: %s, %s, /stairs_detected | Publishing: %s | robot_index=%d timeout_s=%.2f",
      web_teleop_topic_.c_str(), web_action_topic_.c_str(), sim_cmd_vel_topic_.c_str(),
      robot_index_, timeout_s_);
  }

private:
  // ----------------- SIM TUNING (KEEP YOUR SIM VALUES) -----------------
  static constexpr double VX_SCALE   = 2.0;  // forward/back speed multiplier
  static constexpr double VY_SCALE   = 2.0;  // left/right speed multiplier
  static constexpr double VYAW_SCALE = 1.5;  // turning speed multiplier

  // --- Stairs mode behavior (copied feature; you can tune for sim later) ---
  static constexpr double STAIRS_FWD_VX  = 0.15; // m/s (slow!)
  static constexpr double STAIRS_FWD_VY  = 0.00;
  static constexpr double STAIRS_FWD_YAW = 0.00;

  static constexpr double STAIRS_ENTER_HOLD_S = 0.30;
  static constexpr double STAIRS_EXIT_CLEAR_S = 1.50;

  void teleopCb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_vx_ = msg->linear.x;
    latest_vy_ = msg->linear.y;
    latest_vyaw_ = msg->angular.z;
    last_rx_time_ = this->now();
  }

  // NEW: update stairs detection timing (same as Go2)
  void stairsCb(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      last_stairs_true_ = this->now();
    }
  }

  void actionCb(const std_msgs::msg::String::SharedPtr msg) {
    const std::string &a = msg->data;

    // In sim, "stand/sit" may not exist. Keep your existing semantics:
    // - "stop": publish zero cmd_vel immediately
    // - others: log/no-op
    if (a == "stop") {
      // NEW: stop cancels stairs mode immediately (feature parity)
      if (stairs_mode_) {
        stairs_mode_ = false;
        RCLCPP_INFO(this->get_logger(), "[SIM] Stop -> exit STAIRS mode");
      }

      publishZero();
      RCLCPP_INFO(this->get_logger(), "[SIM] Action 'stop' -> zero cmd_vel");
    } else if (a == "stand" || a == "sit") {
      RCLCPP_INFO(this->get_logger(), "[SIM] Action '%s' received (no-op in sim)", a.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "[SIM] Unknown action: '%s'", a.c_str());
    }
  }

  void publishZero() {
    geometry_msgs::msg::Twist t; // zeros by default
    cmd_pub_->publish(t);
  }

  void publishCmd(double vx, double vy, double vyaw) {
    geometry_msgs::msg::Twist t;
    t.linear.x  = vx;
    t.linear.y  = vy;
    t.angular.z = vyaw;
    cmd_pub_->publish(t);
  }

  void tick() {
    const auto now = this->now();

    // --- NEW: stairs mode enter/exit logic (feature parity) ---
    const double stairs_age = (now - last_stairs_true_).seconds();
    const bool stairs_seen_recently = (stairs_age < STAIRS_ENTER_HOLD_S);
    const bool stairs_clear_long_enough = (stairs_age > STAIRS_EXIT_CLEAR_S);

    if (!stairs_mode_ && stairs_seen_recently) {
      stairs_mode_ = true;
      RCLCPP_WARN(this->get_logger(), "[SIM] Entering STAIRS mode (override cmd_vel)");
    }

    if (stairs_mode_ && stairs_clear_long_enough) {
      stairs_mode_ = false;
      RCLCPP_WARN(this->get_logger(), "[SIM] Exiting STAIRS mode");
    }

    // If stairs mode is active, override joystick and just creep forward
    if (stairs_mode_) {
      publishCmd(STAIRS_FWD_VX, STAIRS_FWD_VY, STAIRS_FWD_YAW);
      return;
    }

    // --- Normal teleop watchdog + cmd_vel publish (KEEP SIM behavior) ---
    const double age = (now - last_rx_time_).seconds();

    double vx, vy, vyaw;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      vx = latest_vx_;
      vy = latest_vy_;
      vyaw = latest_vyaw_;
    }

    // Watchdog: no fresh teleop => publish zero
    if (age > timeout_s_) {
      publishZero();
      return;
    }

    const bool stationary =
      (std::abs(vx) < 1e-3 && std::abs(vy) < 1e-3 && std::abs(vyaw) < 1e-3);

    if (stationary) {
      publishZero();
      return;
    }

    // Apply SIM scaling (KEEP)
    const double cmd_vx   = VX_SCALE   * vx;
    const double cmd_vy   = VY_SCALE   * -vy;
    const double cmd_vyaw = VYAW_SCALE * -vyaw;

    publishCmd(cmd_vx, cmd_vy, cmd_vyaw);
  }

  // Params (SIM-UNIQUE)
  int robot_index_{0};
  double timeout_s_{0.25};

  // Topics (SIM-UNIQUE)
  std::string web_teleop_topic_;
  std::string web_action_topic_;
  std::string sim_cmd_vel_topic_;

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stairs_sub_; // NEW
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  std::mutex mtx_;
  double latest_vx_{0.0}, latest_vy_{0.0}, latest_vyaw_{0.0};
  rclcpp::Time last_rx_time_;

  // NEW: stairs mode state
  bool stairs_mode_{false};
  rclcpp::Time last_stairs_true_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebTeleopBridgeSim>());
  rclcpp::shutdown();
  return 0;
}
