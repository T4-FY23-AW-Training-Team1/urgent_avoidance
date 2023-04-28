#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class Tf2ToPoseStamped : public rclcpp::Node
{
public:
  Tf2ToPoseStamped()
  : Node("current_pose_talker")
  {
    subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "~/input/tf", 10, std::bind(&Tf2ToPoseStamped::tf_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/output/pose", 10);
  }

private:
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  for (const auto& transform : msg->transforms)
  {
    // Check if child_frame_id is "base_link"
    if (transform.child_frame_id == "base_link")
    {
      geometry_msgs::msg::PoseStamped pose_stamped;

      // Extract header
      pose_stamped.header = transform.header;

      // Extract transform
      pose_stamped.pose.position.x = transform.transform.translation.x;
      pose_stamped.pose.position.y = transform.transform.translation.y;
      pose_stamped.pose.position.z = transform.transform.translation.z;
      pose_stamped.pose.orientation = transform.transform.rotation;

      // Publish the PoseStamped message
      publisher_->publish(pose_stamped);
    }
  }
}

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tf2ToPoseStamped>());
  rclcpp::shutdown();
  return 0;
}