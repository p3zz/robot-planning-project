#include <functional>
#include <memory>
#include <iostream>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/polygon.hpp"
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


class WallsSubscriber : public rclcpp::Node{
    public:
        WallsSubscriber(Polygon& map_borders) : 
            Node("walls_subscriber"), map_borders{map_borders} {
                subscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
                "map_borders", 10, std::bind(&WallsSubscriber::topic_callback, this, _1));
        }

    private:
        void topic_callback(const geometry_msgs::msg::Polygon & msg) const {
            for(auto p:msg.points){
                map_borders.add_v(Point2D(p.x, p.y));
            }
            std::cout<<"new map borders"<<std::endl;
        }
        rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_;
        Polygon& map_borders;
};

template <typename T>
class SafeValue{
    private:
        T value;
        bool valid;

    public:
        T get(){
            if(valid){
                return value;
            }
        }

        void set(T value){
            value = value;
            valid = true;
        }

};

class ShelfinoDto{
    public:
        ShelfinoDto(){
            gate_position = Point2D(0,0);
            map_borders = Polygon();
        }

        Point2D gate_position;
        Polygon map_borders;
};