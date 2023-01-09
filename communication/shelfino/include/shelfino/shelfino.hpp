#include <functional>
#include <memory>
#include <iostream>
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shapes/shapes.hpp"
#include <thread>

using std::placeholders::_1;

class GatesSubscriber : public rclcpp::Node{
    public:
        GatesSubscriber(Point2D& gate_position) : 
            Node("gates_subscriber"), gate_position{gate_position} {
                subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
                "gate_position", 10, std::bind(&GatesSubscriber::topic_callback, this, _1));
        };

    private:
        void topic_callback(const geometry_msgs::msg::Pose& msg) {
           gate_position = Point2D(msg.position.x, msg.position.y);
            std::cout<<"new gate position"<<std::endl;
        }
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
        Point2D& gate_position;
};