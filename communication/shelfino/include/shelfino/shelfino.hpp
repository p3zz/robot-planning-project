#include <functional>
#include <memory>
#include <iostream>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shapes/shapes.hpp"
#include <thread>

using std::placeholders::_1;

template <typename T>
class SafeValue{
    private:
        T value;
        bool valid;    

    public:
        SafeValue(){}
        SafeValue(T value):value{value}, valid{false}{}

        T get(){
            if(valid){
                return value;
            }
        }

        void set(T v){
            value = v;
            valid = true;
        }

        bool is_valid(){
            return valid;
        }

};

class GatesSubscriber : public rclcpp::Node{
    public:
        GatesSubscriber(SafeValue<Point2D>& gate_position) : 
            Node("gates_subscriber"), gate_position{gate_position} {
                subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
                "gate_position", 10, std::bind(&GatesSubscriber::topic_callback, this, _1));
        };

    private:
        void topic_callback(const geometry_msgs::msg::Pose& msg) {
            gate_position.set(Point2D(msg.position.x, msg.position.y));
            std::cout<<"new gate position"<<std::endl;
        }
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
        SafeValue<Point2D>& gate_position;
};


class WallsSubscriber : public rclcpp::Node{
    public:
        WallsSubscriber(SafeValue<Polygon>& map_borders) : 
            Node("walls_subscriber"), map_borders{map_borders} {
                subscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
                "map_borders", 10, std::bind(&WallsSubscriber::topic_callback, this, _1));
        }

    private:
        void topic_callback(const geometry_msgs::msg::Polygon & msg) const {
            Polygon borders;
            for(auto p:msg.points){
                borders.add_v(Point2D(p.x, p.y));
            }
            map_borders.set(borders);
            std::cout<<"new map borders"<<std::endl;
        }
        rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_;
        SafeValue<Polygon>& map_borders;
};

class ShelfinoDto {
    public:
        ShelfinoDto(){
            gate_position = SafeValue<Point2D>(Point2D(0,0));
            map_borders = SafeValue<Polygon>(Polygon());
        }

        SafeValue<Point2D> gate_position;
        SafeValue<Polygon> map_borders;
};