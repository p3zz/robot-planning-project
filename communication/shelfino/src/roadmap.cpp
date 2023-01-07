#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_msgs/msg/geometry_graph.hpp"

using namespace std::chrono_literals;

// custom_msgs::msg::GeometryGraph graph_from_roadmap(RoadMap map){
//   std_msgs::Header h;
//   const int nodes_len = (int)map.nodes.size();
//   geometry_msgs::Point nodes[nodes_len];
//   custom_msgs::msg::Edges edges[nodes_len];
//   for(int i = 0; i < nodes_len; i++){
//     auto node = map.nodes.at(i);
//     nodes[i] = geometry_msgs::Point(node.x, node.y, 0);
//     std::vector<Point2D> neighbors;
    
//   }
// }

class RoadmapPublisher : public rclcpp::Node
{
  public:
    RoadmapPublisher()
    : Node("roadmap_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<custom_msgs::msg::GeometryGraph>("roadmap", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&RoadmapPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = custom_msgs::msg::GeometryGraph();
    //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_msgs::msg::GeometryGraph>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoadmapPublisher>());
  rclcpp::shutdown();
  return 0;
}