#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<nav_msgs::msg::Path>(
      "plan", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const nav_msgs::msg::Path & msg) const
    {
      auto start = msg.poses[0].pose.position;
      auto end = msg.poses[1].pose.position;
      RCLCPP_INFO(this->get_logger(), "Received path [(%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f)]", start.x, start.y, start.z, end.x, end.y, end.z);
    }
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}