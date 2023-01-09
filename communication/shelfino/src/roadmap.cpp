#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_msgs/msg/geometry_graph.hpp"

using namespace std::chrono_literals;

class RoadmapPublisher : public rclcpp::Node
{
  public:
    RoadmapPublisher(): Node("roadmap_publisher"), count_(0) {
      publisher_ = this->create_publisher<custom_msgs::msg::GeometryGraph>("roadmap", 10);
      auto message = custom_msgs::msg::GeometryGraph();
      publisher_->publish(message);
    }

  private:
    rclcpp::Publisher<custom_msgs::msg::GeometryGraph>::SharedPtr publisher_;
    size_t count_;
};
