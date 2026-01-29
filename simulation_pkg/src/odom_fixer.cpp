#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomFixer : public rclcpp::Node
{
public:
  OdomFixer() : Node("odom_fixer")
  {
    declare_parameter<std::string>("in_topic", "/diff_controller/odom");
    declare_parameter<std::string>("out_topic", "/odom_fixed");
    declare_parameter<std::string>("odom_frame_id", "odom");
    declare_parameter<std::string>("base_frame_id", "base_link");

    in_topic_ = get_parameter("in_topic").as_string();
    out_topic_ = get_parameter("out_topic").as_string();
    odom_frame_id_ = get_parameter("odom_frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();

    pub_ = create_publisher<nav_msgs::msg::Odometry>(out_topic_, rclcpp::QoS(10));
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      in_topic_, rclcpp::QoS(10),
      std::bind(&OdomFixer::cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "odom_fixer: %s -> %s (frame_id=%s child_frame_id=%s)",
                in_topic_.c_str(), out_topic_.c_str(),
                odom_frame_id_.c_str(), base_frame_id_.c_str());
  }

private:
  void cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry out = *msg;
    out.header.frame_id = odom_frame_id_;
    out.child_frame_id = base_frame_id_;
    pub_->publish(out);
  }

  std::string in_topic_, out_topic_, odom_frame_id_, base_frame_id_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomFixer>());
  rclcpp::shutdown();
  return 0;
}
