#include "shelfino/shelfino.hpp"

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

ObstaclesSubscriber::ObstaclesSubscriber(std::optional<std::vector<Polygon>>& obstacles) : Node("obstacles_subscriber"), obstacles{obstacles} {
    subscription_ = this->create_subscription<custom_msgs::msg::ObstacleArrayMsg>(
    "obstacles", 10, std::bind(&ObstaclesSubscriber::topic_callback, this, _1));
}

void ObstaclesSubscriber::topic_callback(const custom_msgs::msg::ObstacleArrayMsg & msg) {
    obstacles.emplace(obstacles_from_msg(msg));
    for(auto &ob: obstacles.value()){
        RCLCPP_INFO(this->get_logger(), "Obstacle received");
        for(auto &v: ob.vertexes){  
           RCLCPP_INFO(this->get_logger(), "Vertex: (%f, %f)", v.x, v.y);
        }
    }
}