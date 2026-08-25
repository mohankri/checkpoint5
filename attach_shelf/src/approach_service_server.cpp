#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <cstddef>
#include <vector>

using namespace std;

typedef struct leg {
  double x;
  double y;
} leg_t;

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
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 100);

    elev_publisher_ =
        this->create_publisher<std_msgs::msg::String>("/elevator_up", 100);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos,
        std::bind(&ApproachService::odom_callback, this,
                  std::placeholders::_1));
  }

private:
  void handle_approach_service(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {

    // attach_to_shelf_ = request->attach_shelf;

    /*
        Start the timer if attach to shelf is true
    */

    if (request->attach_to_shelf) {
      phase_ = Phase::APPROACH;
      auto timer_period = std::chrono::milliseconds(100);

      drive_timer_ = this->create_wall_timer(
          timer_period, std::bind(&ApproachService::timer_callback, this));
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    current_x_ = odom_msg->pose.pose.position.x;
    current_y_ = odom_msg->pose.pose.position.y;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    vector<size_t> current_;
    vector<vector<size_t>> cluster_;

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

    auto leg_point = [&](const std::vector<size_t> &c) {
      auto mid_index = c[c.size() / 2];
      /* pick the angle of mid_index from ranges[] */
      auto angle = msg->angle_min + mid_index * msg->angle_increment;
      auto dist = msg->ranges[mid_index];
      /* Get X*/
      auto x = dist * std::cos(angle);
      /* Get y */
      auto y = dist * std::sin(angle);

      return leg_t{x, y};
    };

    if (cluster_.size() >= 2) {
      leg_t left_leg = leg_point(cluster_.front());
      leg_t right_leg = leg_point(cluster_.back());

      leg_t target =
          leg_t{(left_leg.x + right_leg.x) / 2, (right_leg.y + left_leg.y) / 2};

      publish_transform(msg, target);
    }
  }

  void publish_transform(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                         leg_t &target) {
    // in the scan callback, after computing the midpoint `target` (body frame):
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp; // the scan's own time, not now()
    t.header.frame_id =
        "robot_front_laser_base_link"; // the frame `target` was computed IN
    t.child_frame_id = "cart_frame";   // the frame being created
    t.transform.translation.x = target.x;
    t.transform.translation.y = target.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation.w = 1.0; // identity (x=y=z=0 default) — position-only
    tf_broadcaster_->sendTransform(t);
  }

  void timer_callback() {
    geometry_msgs::msg::Twist cmd;

    geometry_msgs::msg::TransformStamped t_;

    try {
      t_ = tf_buffer_->lookupTransform("robot_base_link", "cart_frame",
                                       tf2::TimePointZero);
    } catch (const tf2::TransformException &e) {
      // RCLCPP_INFO(this->get_logger(), "Exception Received %s", e.what());
      cmd.linear.x = 0.1; // linear velocity
      cmd.angular.z = 0;  // angular velociy

      vel_publisher_->publish(cmd);
      return;
    }

    double dx = t_.transform.translation.x;
    double dy = t_.transform.translation.y;

    switch (phase_) {
    case Phase::APPROACH: {
      /* calculate the distance */
      double error_distance = sqrt(dx * dx + dy * dy);
      /* calculate the yaw */
      double error_yaw = atan2(dy, dx);

      if (abs(error_yaw) < 0.02) {
        cmd.angular.z = 0; // angular velociy
      } else {
        error_yaw = std::clamp(error_yaw, -0.02, 0.02);
        cmd.angular.z = error_yaw; // angular velociy
      }
      cmd.linear.x = 0.1; // linear velocity

      if (error_distance < min_distance_) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        phase_ = Phase::ADVANCE;
        start_x_ = current_x_;
        start_y_ = current_y_;
        RCLCPP_INFO(this->get_logger(), "Move to Advance");
      }
      // RCLCPP_INFO(this->get_logger(), "distance %f error_yaw %f",
      //             error_distance, error_yaw);

      vel_publisher_->publish(cmd);

      break;
    }
    case Phase::ADVANCE: {
      double traveled =
          std::hypot(current_x_ - start_x_, current_y_ - start_y_);
      cmd.linear.x = 0.2;

      if ((0.60 - traveled) < 0.01) {
        cmd.linear.x = 0.0;
        phase_ = Phase::ELEVATOR_UP;
      }
      cmd.angular.z = 0.0;
      vel_publisher_->publish(cmd);
      break;
    }
    case Phase::ELEVATOR_UP: {
      std_msgs::msg::String lift;
      lift.data = "up";
      elev_publisher_->publish(lift);
      phase_ = Phase::DONE;
      break;
    }
    case Phase::DONE: {
      // RCLCPP_INFO(this->get_logger(), "Pre Approach Complete");
      break;
    }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elev_publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::TimerBase::SharedPtr drive_timer_;
  const double min_distance_ = 0.30;
  rclcpp::Time advance_start_;

  double current_x_ = 0.0, current_y_ = 0.0; // odom frame
  double start_x_ = 0.0, start_y_ = 0.0;     // captured at phase entry

  const std::string service_name_ = "/approach_shelf";

  enum class Phase { APPROACH, ADVANCE, ELEVATOR_UP, DONE };
  Phase phase_ = Phase::DONE;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}