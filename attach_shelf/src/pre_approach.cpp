#include "rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos_overriding_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
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

    RCLCPP_INFO(this->get_logger(), "Obstacle Distance %f",
                this->get_parameter("obstacle").as_double());
    RCLCPP_INFO(this->get_logger(), "Degrees %f",
                this->get_parameter("degrees").as_double());
  }

protected:
  CallbackReturn on_configure(rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "On Configure");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "On Activate");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "On Deactivate");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "On Cleanup");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "On Shutdown");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "On Error");
    return CallbackReturn::SUCCESS;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PreApproach>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
}