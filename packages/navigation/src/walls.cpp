#include "navigation/navigation.hpp"

Polygon borders_from_msg(geometry_msgs::msg::Polygon msg){
    Polygon borders;
    for(auto p:msg.points){
        borders.add_v(Point2D(p.x, p.y));
    }
    return borders;
}

WallsSubscriber::WallsSubscriber(std::optional<Polygon>& map_borders) : 
    Node("walls_subscriber"), map_borders{map_borders} {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        subscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
        "map_borders", qos, std::bind(&WallsSubscriber::topic_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Waiting");
}

void WallsSubscriber::topic_callback(const geometry_msgs::msg::Polygon & msg) {
    if(map_borders.has_value()){
        return;
    }
    map_borders.emplace(borders_from_msg(msg));
    RCLCPP_INFO(this->get_logger(), "Borders received");
    for(auto &v: map_borders.value().vertexes){
        RCLCPP_INFO(this->get_logger(), "Border: (%f, %f)", v.x, v.y);
    }
}