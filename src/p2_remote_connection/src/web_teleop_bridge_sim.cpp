#include <chrono>
#include <string>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

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

    // Topics
    web_teleop_topic_ = "/web_teleop";
    web_action_topic_ = "/web_action";
    sim_cmd_vel_topic_ = "/robot" + std::to_string(robot_index_) + "/cmd_vel";

    teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      web_teleop_topic_, 10,
      std::bind(&WebTeleopBridgeSim::teleopCb, this, std::placeholders::_1));

    action_sub_ = this->create_subscription<std_msgs::msg::String>(
      web_action_topic_, 10,
      std::bind(&WebTeleopBridgeSim::actionCb, this, std::placeholders::_1));

    // Publish to sim cmd_vel
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(sim_cmd_vel_topic_, 10);

    // 50 ms ~= 20 Hz output stream (same rate you used before)
    timer_ = this->create_wall_timer(50ms, std::bind(&WebTeleopBridgeSim::tick, this));

    last_rx_time_ = this->now();
    RCLCPP_INFO(this->get_logger(),
      "WebTeleopBridgeSim started. Subscribing: %s, %s | Publishing: %s | robot_index=%d timeout_s=%.2f",
      web_teleop_topic_.c_str(), web_action_topic_.c_str(), sim_cmd_vel_topic_.c_str(),
      robot_index_, timeout_s_);
  }

private:
 // ----------------- TUNE THESE -----------------
  static constexpr double VX_SCALE   = 2.0;  // forward/back speed multiplier
  static constexpr double VY_SCALE   = 2.0;  // left/right speed multiplier
  static constexpr double VYAW_SCALE = 1.5;  // turning speed multiplier

  void teleopCb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_vx_ = msg->linear.x;
    latest_vy_ = msg->linear.y;
    latest_vyaw_ = msg->angular.z;
    last_rx_time_ = this->now();
  }

  void actionCb(const std_msgs::msg::String::SharedPtr msg) {
    const std::string &a = msg->data;

    // In sim, "stand/sit" may not exist. We implement sensible behavior:
    // - "stop": publish zero cmd_vel immediately
    // - others: just log (or you can add sim-specific actions later)
    if (a == "stop") {
      publishZero();
      RCLCPP_INFO(this->get_logger(), "[SIM] Action 'stop' -> zero cmd_vel");
    } else if (a == "stand" || a == "sit") {
      // Most sims don't need explicit stand/sit; the RL locomotion runs continuously.
      RCLCPP_INFO(this->get_logger(), "[SIM] Action '%s' received (no-op in sim)", a.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "[SIM] Unknown action: '%s'", a.c_str());
    }
  }

  void publishZero() {
    geometry_msgs::msg::Twist t;
    // all zeros by default
    cmd_pub_->publish(t);
  }

  void tick() {
    const double age = (this->now() - last_rx_time_).seconds();

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

    geometry_msgs::msg::Twist t;
    if (!stationary) {
      t.linear.x  = VX_SCALE   * vx;  
      t.linear.y  = VY_SCALE   * -vy;
      t.angular.z = VYAW_SCALE * -vyaw;
    }
    // If stationary: publish zeros (default)

    cmd_pub_->publish(t);
  }

  // Params
  int robot_index_{0};
  double timeout_s_{0.25};

  // Topics
  std::string web_teleop_topic_;
  std::string web_action_topic_;
  std::string sim_cmd_vel_topic_;

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  std::mutex mtx_;
  double latest_vx_{0.0}, latest_vy_{0.0}, latest_vyaw_{0.0};
  rclcpp::Time last_rx_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebTeleopBridgeSim>());
  rclcpp::shutdown();
  return 0;
}
