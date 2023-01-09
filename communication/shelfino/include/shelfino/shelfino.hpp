#include <functional>
#include <memory>
#include <iostream>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shapes/shapes.hpp"
#include "custom_msgs/msg/obstacle_array_msg.hpp"
#include "custom_msgs/msg/obstacle_msg.hpp"
#include <thread>

using std::placeholders::_1;

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

Polygon borders_from_msg(geometry_msgs::msg::Polygon msg){
    Polygon borders;
    for(auto p:msg.points){
        borders.add_v(Point2D(p.x, p.y));
    }
    return borders;
}

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

class ShelfinoDto {
    public:
        ShelfinoDto(){
            gate_position = SafeValue<Point2D>(Point2D(0,0));
            map_borders = SafeValue<Polygon>(Polygon());
            obstacles = SafeValue<std::vector<Polygon>>();
        }

        SafeValue<Point2D> gate_position;
        SafeValue<Polygon> map_borders;
        SafeValue<std::vector<Polygon>> obstacles;
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
            map_borders.set(borders_from_msg(msg));
            std::cout<<"new map borders"<<std::endl;
        }
        rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_;
        SafeValue<Polygon>& map_borders;
};

class ObstaclesSubscriber : public rclcpp::Node{
public:
    ObstaclesSubscriber(SafeValue<std::vector<Polygon>>& obstacles) : Node("obstacles_subscriber"), obstacles{obstacles} {
        subscription_ = this->create_subscription<custom_msgs::msg::ObstacleArrayMsg>(
        "obstacles", 10, std::bind(&ObstaclesSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const custom_msgs::msg::ObstacleArrayMsg & msg) const {
        obstacles.set(obstacles_from_msg(msg));
    }
    rclcpp::Subscription<custom_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_;
    SafeValue<std::vector<Polygon>>& obstacles;
};
