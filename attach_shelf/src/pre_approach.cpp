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

#include <cstdint>
#include <memory>

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

    /* declare decriptor */
    this->declare_parameter<double>("obstacle", 0.0, obstacle_desc);
    this->declare_parameter<double>("degrees", 0.0, degrees_desc);

    init_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(0), [this]() {
          RCLCPP_INFO(get_logger(), "init timer fired"); // ← does THIS print?

          this->configure(); // node fully constructed + spinning now
          this->activate();
          init_timer_->cancel(); // one-shot
        });

    RCLCPP_INFO(this->get_logger(), "Obstacle Distance %f",
                this->get_parameter("obstacle").as_double());
    RCLCPP_INFO(this->get_logger(), "Degrees %f",
                this->get_parameter("degrees").as_double());
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

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "On Activate");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &prev_state) {
    RCLCPP_INFO(this->get_logger(), "On Deactivate");
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
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {}

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {}

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr init_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PreApproach>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
}