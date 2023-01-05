#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/obstacle_array_msg.hpp"
#include "shapes/shapes.hpp"

using std::placeholders::_1;

std::vector<Polygon> obstacles_from_msg(custom_msgs::msg::ObstacleArrayMsg msg){
  std::vector<Polygon> obstacles;
  for(auto obstacle: msg.obstacles){
    Polygon p;
    for(auto point: obstacle.polygon.points){
      p.add_v(Point2D(point.x, point.y));
    }
    obstacles.push_back(p);
  }
  return obstacles;
} 

class MinimalSubscriber : public rclcpp::Node{
public:
  MinimalSubscriber() : Node("obstacles_subscriber") {
    subscription_ = this->create_subscription<custom_msgs::msg::ObstacleArrayMsg>(
      "obstacles", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const custom_msgs::msg::ObstacleArrayMsg & msg) const {
    auto obstacles = obstacles_from_msg(msg);
    // RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.num << "'");
  }
  rclcpp::Subscription<custom_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}