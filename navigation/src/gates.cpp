#include "navigation/navigation.hpp"

GatesSubscriber::GatesSubscriber(std::optional<std::vector<Point2D>>& gates_position) : 
    Node("gates_subscriber"), gates_position{gates_position} {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "gate_position", qos, std::bind(&GatesSubscriber::topic_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Waiting");
}

void GatesSubscriber::topic_callback(const geometry_msgs::msg::PoseArray& msg) {
    if(gates_position.has_value()){
        return;
    }
    std::vector<Point2D> gates;
    for(auto &pose: msg.poses){
        gates.push_back(Point2D(pose.position.x, pose.position.y));
    }
    gates_position.emplace(gates);
    for(auto &gate: gates_position.value()){
        RCLCPP_INFO(this->get_logger(), "Gate received: (%f, %f)", gate.x, gate.y);
    }
}