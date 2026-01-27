#include <chrono>
#include <string>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "unitree_api/msg/request.hpp"
#include "p2_remote_connection/common/ros2_sport_client.h"

using namespace std::chrono_literals;

class WebTeleopBridge : public rclcpp::Node {
public:
  WebTeleopBridge()
  : Node("web_teleop_bridge"),
    sportClient_(this)
  {
    teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/web_teleop", 10,
      std::bind(&WebTeleopBridge::teleopCb, this, std::placeholders::_1));

    action_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/web_action", 10,
      std::bind(&WebTeleopBridge::actionCb, this, std::placeholders::_1));

    // NEW: stairs detection subscriber (published by your stairs detector node)
    stairs_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/stairs_detected", 10,
      std::bind(&WebTeleopBridge::stairsCb, this, std::placeholders::_1));

    // 50 ms ~= 20 Hz command stream (tune as needed)
    timer_ = this->create_wall_timer(50ms, std::bind(&WebTeleopBridge::tick, this));

    last_rx_time_ = this->now();
    last_stairs_true_ = this->now() - rclcpp::Duration::from_seconds(1000.0);

    RCLCPP_INFO(this->get_logger(), "WebTeleopBridge started.");
  }

private:
  // --- Teleop scaling ---
  static constexpr double VX_SCALE   = 1.0;   // forward/back speed multiplier
  static constexpr double VY_SCALE   = 1.0;   // left/right speed multiplier
  static constexpr double VYAW_SCALE = 1.5;   // turning speed multiplier

  // --- Stairs mode behavior (tune these) ---
  static constexpr float STAIRS_FWD_VX  = 0.15f; // m/s (slow!)
  static constexpr float STAIRS_FWD_VY  = 0.00f;
  static constexpr float STAIRS_FWD_YAW = 0.00f;

  static constexpr double STAIRS_ENTER_HOLD_S = 0.30; // must see stairs "recently" to enter
  static constexpr double STAIRS_EXIT_CLEAR_S = 1.50; // must be clear this long to exit

  void teleopCb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "rx /web_teleop vx=%.2f vy=%.2f wz=%.2f",
      msg->linear.x, msg->linear.y, msg->angular.z);

    std::lock_guard<std::mutex> lk(mtx_);
    latest_vx_ = msg->linear.x;
    latest_vy_ = msg->linear.y;
    latest_vyaw_ = msg->angular.z;
    last_rx_time_ = this->now();
  }

  // NEW: update stairs detection timing
  void stairsCb(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      last_stairs_true_ = this->now();
    }
  }

  void actionCb(const std_msgs::msg::String::SharedPtr msg) {
    const std::string &a = msg->data;

    if (a == "stand") {
      sportClient_.StandUp(req_);
    } else if (a == "sit") {
      sportClient_.StandDown(req_);
    } else if (a == "stop") {
      // NEW: stop cancels stairs mode immediately
      if (stairs_mode_) {
        stairs_mode_ = false;
        sportClient_.WalkUpright(req_, false);
      }
      sportClient_.StopMove(req_);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown action: '%s'", a.c_str());
    }
  }

  bool was_active_{false};          // tracks whether we were actively sending Move()
  static constexpr double DEADBAND = 0.05;  // joystick deadband (tune 0.03..0.10)

  void tick() {
    const auto now = this->now();
    if (!was_active_) {
      sportClient_.StandUp(req_);           // optional
      sportClient_.WalkUpright(req_, true); // IMPORTANT test
    }

    // // --- stairs mode enter/exit logic ---
    // const double stairs_age = (now - last_stairs_true_).seconds();
    // const bool stairs_seen_recently = (stairs_age < STAIRS_ENTER_HOLD_S);
    // const bool stairs_clear_long_enough = (stairs_age > STAIRS_EXIT_CLEAR_S);

    // if (!stairs_mode_ && stairs_seen_recently) {
    //   stairs_mode_ = true;
    //   RCLCPP_WARN(this->get_logger(), "Entering STAIRS mode: WalkUpright(true)");
    //   sportClient_.WalkUpright(req_, true);
    // }

    // if (stairs_mode_ && stairs_clear_long_enough) {
    //   stairs_mode_ = false;
    //   RCLCPP_WARN(this->get_logger(), "Exiting STAIRS mode: WalkUpright(false)");
    //   sportClient_.WalkUpright(req_, false);
    // }

    // // If stairs mode is active, override joystick and just creep forward
    // if (stairs_mode_) {
    //   sportClient_.Move(req_, STAIRS_FWD_VX, STAIRS_FWD_VY, STAIRS_FWD_YAW);
    //   return;
    // }

    // --- Normal teleop watchdog + teleop motion ---
    const double timeout_s = 0.25; // 250 ms watchdog
    const double age = (now - last_rx_time_).seconds();

    double vx, vy, vyaw;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      vx = latest_vx_;
      vy = latest_vy_;
      vyaw = latest_vyaw_;
    }

    // NEW: deadband
    auto dz = [](double v) -> double {
      return (std::abs(v) < DEADBAND) ? 0.0 : v;
    };
    vx   = dz(vx);
    vy   = dz(vy);
    vyaw = dz(vyaw);
  
    const bool timed_out = (age > timeout_s);
  
    const bool stationary =
      (std::abs(vx) < 1e-6 && std::abs(vy) < 1e-6 && std::abs(vyaw) < 1e-6);
  
    const bool active_now = (!timed_out) && (!stationary);
  
    // NEW: if not active, send StopMove only ONCE when transitioning from active->inactive
    if (!active_now) {
      if (was_active_) {
        sportClient_.StopMove(req_);
      }
      was_active_ = false;
      return; // go quiet
    }
  
    // Active: send Move
    was_active_ = true;
  
    const float cmd_vx   = static_cast<float>(VX_SCALE * vx);
    const float cmd_vy   = static_cast<float>(VY_SCALE * -vy);
    const float cmd_vyaw = static_cast<float>(VYAW_SCALE * -vyaw);
  
    sportClient_.Move(req_, cmd_vx, cmd_vy, cmd_vyaw);
  }

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stairs_sub_; // NEW
  rclcpp::TimerBase::SharedPtr timer_;

  // Unitree
  unitree_api::msg::Request req_;
  SportClient sportClient_;

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
  rclcpp::spin(std::make_shared<WebTeleopBridge>());
  rclcpp::shutdown();
  return 0;
}
