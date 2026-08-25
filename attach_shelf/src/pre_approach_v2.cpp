#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos_overriding_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

#include <cstdint>
#include <memory>

using namespace std;

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class PreApproach : public rclcpp_lifecycle::LifecycleNode {
public:
  PreApproach() : rclcpp_lifecycle::LifecycleNode("PreApproachNode") {
    RCLCPP_INFO(this->get_logger(), "Pre Approach");

    /* parameter descriptor */
    rcl_interfaces::msg::ParameterDescriptor obstacle_desc;
    obstacle_desc.description = "Obstacle distance in meter.";

    rcl_interfaces::msg::ParameterDescriptor degrees_desc;
    degrees_desc.description = "Number of degree of rotation";

    rcl_interfaces::msg::ParameterDescriptor final_approach_desc;
    final_approach_desc.description = "Final Approach description";

    /* declare decriptor */
    this->declare_parameter<double>("obstacle", 0.0, obstacle_desc);
    this->declare_parameter<int>("degrees", 0.0, degrees_desc);
    this->declare_parameter<bool>("final_approach", false, final_approach_desc);

    init_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(0), [this]() {
          RCLCPP_INFO(get_logger(), "init timer fired"); // ← does THIS print?

          this->configure(); // node fully constructed + spinning now
          this->activate();
          init_timer_->cancel(); // one-shot
        });

    service_client_ =
        this->create_client<attach_shelf::srv::GoToLoading>(service_name_);
    // Wait for the service to be available (checks every second)
    while (!service_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
    }
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "On Configure");
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 100);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos,
        std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos,
        std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));
    obstacle_range_ = this->get_parameter("obstacle").as_double();
    degrees_rotation_ = this->get_parameter("degrees").as_int();
    final_approach_ = this->get_parameter("final_approach").as_bool();

    RCLCPP_INFO(this->get_logger(), "Final Approach %d", final_approach_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "On Activate");
    auto timer_period = std::chrono::milliseconds(100);
    phase_ = Phase::APPROACH;
    drive_timer_ = this->create_wall_timer(
        timer_period, std::bind(&PreApproach::timer_callback, this));
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "On Deactivate");
    drive_timer_->cancel();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "On Cleanup");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "On Shutdown");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "On Error");
    return CallbackReturn::SUCCESS;
  }

private:
  void attach_to_shelf_response(
      rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Attach to Self Response %d",
                response->complete);
  }

  void send_attach_to_shelf_request(bool flag) {
    auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();

    request->attach_to_shelf = flag;

    // Callback fires when the response arrives — no blocking, no second
    // executor
    service_client_->async_send_request(
        request, std::bind(&PreApproach::attach_to_shelf_response, this,
                           std::placeholders::_1));
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // float center_min = std::numeric_limits<float>::infinity();
    // float center_max = 0.0f;

    error_distance_ = obstacle_range_ *
                      (msg->ranges[msg->ranges.size() / 2] - obstacle_range_);

    if (msg->ranges[msg->ranges.size() / 2] < obstacle_range_) {
      obstacle_ahead_ = true;
      phase_ = Phase::ROTATE;
    } else {
      obstacle_ahead_ = false;
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    tf2::Quaternion q(
        odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);

    current_yaw_ = 2.0 * std::atan2(q.z(), q.w());
    if (target_yaw_ == 0) {
      target_yaw_ = degrees_rotation_ * M_PI / 180 + current_yaw_;
    }
  }

  void timer_callback() {
    geometry_msgs::msg::Twist cmd;

    switch (phase_) {
    case Phase::APPROACH: {
      auto target_speed = 0.04;
      cmd.linear.x = target_speed + error_distance_; // linear
      cmd.angular.z = 0;
      vel_pub_->publish(cmd);
      break;
    }
    case Phase::ROTATE: {
      auto err = target_yaw_ - current_yaw_;
      err = std::atan2(std::sin(err),
                       std::cos(err)); // wrap: −3.15 target handled

      // err =
      cmd.linear.x = 0.0;
      cmd.angular.z = std::clamp(err, -0.05, 0.05);
      ; // err;
#if 1
      RCLCPP_INFO(
          this->get_logger(),
          "Perform ROTATE Operation target %0.2f current %0.2f diff %0.2f",
          target_yaw_, current_yaw_, std::clamp(err, -0.03, 0.03));
#endif
      vel_pub_->publish(cmd);
      if (abs(err) < 0.03) {
        phase_ = Phase::DONE;
        send_attach_to_shelf_request(final_approach_);
      }
      break;
    }
    case Phase::DONE: {
      RCLCPP_INFO(this->get_logger(), "Pre Approach Complete");
      vel_pub_->publish(cmd);
      this->deactivate();
      this->cleanup();
      break;
    }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr service_client_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr drive_timer_;

  double obstacle_range_ = 0.0f;
  int degrees_rotation_ = 0;
  bool obstacle_ahead_ = false;
  double error_distance_ = 0.0f;
  double current_yaw_ = 0.0f;
  double target_yaw_ = 0.0f;
  bool final_approach_ = false;
  const std::string service_name_ = "/approach_shelf";

  enum class Phase { APPROACH, ROTATE, DONE };
  Phase phase_ = Phase::DONE;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PreApproach>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
}