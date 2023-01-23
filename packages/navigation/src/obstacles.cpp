#include "navigation/navigation.hpp"

std::vector<Polygon> obstacles_from_msg(obstacles_msgs::msg::ObstacleArrayMsg msg){
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
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "obstacles", qos, std::bind(&ObstaclesSubscriber::topic_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Waiting");
}

void ObstaclesSubscriber::topic_callback(const obstacles_msgs::msg::ObstacleArrayMsg & msg) {
    if(obstacles.has_value()){
      return;
    }
    obstacles.emplace(obstacles_from_msg(msg));
    for(auto &ob: obstacles.value()){
        RCLCPP_INFO(this->get_logger(), "Obstacle received");
        for(auto &v: ob.vertexes){  
           RCLCPP_INFO(this->get_logger(), "Vertex: (%f, %f)", v.x, v.y);
        }
    }
}