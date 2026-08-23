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