#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanFixer : public rclcpp::Node
{
public:
  ScanFixer()
  : Node("scan_fixer")
  {
    // Parameters
    this->declare_parameter<std::string>("in_topic", "/scan");
    this->declare_parameter<std::string>("out_topic", "/scan_fixed");
    this->declare_parameter<bool>("override_stamp", true);

    in_topic_ = this->get_parameter("in_topic").as_string();
    out_topic_ = this->get_parameter("out_topic").as_string();
    override_stamp_ = this->get_parameter("override_stamp").as_bool();

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(out_topic_, rclcpp::QoS(10));

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      in_topic_,
      rclcpp::QoS(10),
      std::bind(&ScanFixer::callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(),
      "scan_fixer running. in_topic='%s' out_topic='%s' override_stamp=%s",
      in_topic_.c_str(), out_topic_.c_str(), override_stamp_ ? "true" : "false");
  }

private:
  void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Copy message
    sensor_msgs::msg::LaserScan out = *msg;

    // Use node time (sim time if use_sim_time=true) for consistent stamps if requested
    const rclcpp::Time now = this->get_clock()->now();
    if (override_stamp_) {
      out.header.stamp = now;
    }

    // Estimate scan_time from consecutive callbacks using "now"
    double scan_time = 0.0;
    if (last_time_.has_value()) {
      const double dt = (now - last_time_.value()).seconds();
      if (dt > 0.0) {
        scan_time = dt;
      }
    }
    last_time_ = now;

    out.scan_time = static_cast<float>(scan_time);

    const std::size_t n = out.ranges.size();
    if (n > 1 && scan_time > 0.0) {
      out.time_increment = static_cast<float>(scan_time / static_cast<double>(n - 1));
    } else {
      out.time_increment = 0.0f;
    }

    pub_->publish(out);
  }

  std::string in_topic_;
  std::string out_topic_;
  bool override_stamp_{true};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;

  std::optional<rclcpp::Time> last_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanFixer>());
  rclcpp::shutdown();
  return 0;
}
