#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("obstacles_subscriber")
  {
    subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "obstacles", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const obstacles_msgs::msg::ObstacleArrayMsg & msg) const
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.num << "'");
  }
  rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}