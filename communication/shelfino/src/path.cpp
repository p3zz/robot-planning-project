#include "shelfino/shelfino.hpp"

PathPublisher::PathPublisher(std::optional<DubinCurve>& path, std::string topic_name) : Node("path_publisher"), path{path} {    
    publisher_ = this->create_publisher<nav_msgs::msg::Path>(topic_name, 10);
    timer_ = this->create_wall_timer(1s, std::bind(&PathPublisher::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Ready to publish");
}

void PathPublisher::timer_callback(){
    if(!path.has_value()){
        return;
    }
    std_msgs::msg::Header h;
    h.stamp = this->get_clock()->now();
    auto msg = msg_from_curve(path.value(), h);
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing");
}
