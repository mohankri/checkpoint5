#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <vector>

using namespace std;

class ApproachService : public rclcpp::Node {
public:
  ApproachService() : rclcpp::Node("ApproachService") {
    service_ = this->create_service<attach_shelf::srv::GoToLoading>(
        service_name_,
        std::bind(&ApproachService::handle_approach_service, this,
                  std::placeholders::_1, std::placeholders::_2));

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos,
        std::bind(&ApproachService::scan_callback, this,
                  std::placeholders::_1));
    // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  void handle_approach_service(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Approach Service Called");
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    vector<int> current_;
    vector<vector<int>> cluster_;

    for (size_t i = 0; i < msg->intensities.size(); i++) {
      if (msg->intensities[i] >= 8000) {
        current_.push_back(i);
      } else if (!current_.empty()) {
        cluster_.push_back(current_);
        current_.clear();
      }
    }
    if (!current_.empty()) {
      cluster_.push_back(current_);
      current_.clear();
    }
    RCLCPP_INFO(this->get_logger(), "Cluster Size %ld", cluster_.size());
    if (cluster_.size() == 0) {
    }
  }

  void publish_transform() {
#if 0
    // in the scan callback, after computing the midpoint `target` (body frame):
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp; // the scan's own time, not now()
    t.header.frame_id =
        "robot_front_laser_base_link"; // the frame `target` was computed IN
    t.child_frame_id = "cart_frame";   // the frame being created
    t.transform.translation.x = target.x;
    t.transform.translation.y = target.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation.w =
        1.0; // identity (x=y=z=0 default) — position-only frame
    tf_broadcaster_->send_transform(t);
#endif
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
  // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  const std::string service_name_ = "/approach_shelf";
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}