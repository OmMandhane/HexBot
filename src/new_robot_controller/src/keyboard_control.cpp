#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/header.hpp"

class TwistToTwistStamped : public rclcpp::Node
{
public:
  TwistToTwistStamped()
  : Node("twist_to_twist_stamped")
  {
    // Create a subscriber to the Twist topic
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10,
      std::bind(&TwistToTwistStamped::twist_callback, this, std::placeholders::_1));

    // Create a publisher for the TwistStamped topic
    twist_stamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/new_robot_controller/cmd_vel",
      10);
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Create and populate a TwistStamped message
    auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
    twist_stamped_msg.header.stamp = this->now();
    twist_stamped_msg.header.frame_id = "base_link";
    twist_stamped_msg.twist = *msg;

    // Publish the TwistStamped message
    twist_stamped_pub_->publish(twist_stamped_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToTwistStamped>());
  rclcpp::shutdown();
  return 0;
}
