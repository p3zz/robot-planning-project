#include <functional>
#include <memory>
#include <iostream>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "custom_msgs/msg/obstacle_array_msg.hpp"
#include "custom_msgs/msg/obstacle_msg.hpp"
#include "custom_msgs/msg/geometry_graph.hpp"
#include "custom_msgs/msg/edges.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "shapes/shapes.hpp"
#include "dubins/dubins.hpp"
#include "map/map.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

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

custom_msgs::msg::GeometryGraph msg_from_roadmap(RoadMap rm, std_msgs::msg::Header h){
  custom_msgs::msg::GeometryGraph gg;
  std::vector<custom_msgs::msg::Edges> nodes_edges;
  std::vector<geometry_msgs::msg::Point> nodes;
  for(auto &node: rm.getNodes()){
    geometry_msgs::msg::Point p;
    p.x = node.x;
    p.y = node.y;
    p.z = 0;
    nodes.push_back(p);
    std::vector<unsigned int> neighbors;
    for(auto &link: rm.getLinks()){
      Point2D neighbor;
      if(node == link.node1){
        neighbor = link.node2;
      }
      else if(node == link.node2){
        neighbor = link.node1;
      }
      auto i = rm.get_node_index(neighbor);
      if(i > -1){
        neighbors.push_back(i);
      }
    }
    custom_msgs::msg::Edges edges;
    edges.node_ids = neighbors;
    nodes_edges.push_back(edges);
  }

  gg.header = h;
  gg.nodes = nodes;
  gg.edges = nodes_edges;

  return gg;
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
            pose = SafeValue<DubinPoint>(DubinPoint(0,0,0));
        }

        SafeValue<Point2D> gate_position;
        SafeValue<Polygon> map_borders;
        SafeValue<std::vector<Polygon>> obstacles;
        SafeValue<DubinPoint> pose;
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

class PoseListener : public rclcpp::Node {
public:
  PoseListener(SafeValue<DubinPoint>& pose): Node("shelfino2_listener"), pose{pose}{
    target_frame_ = this->declare_parameter<std::string>("target_frame", "shelfino2/base_link");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(1s, std::bind(&PoseListener::timer_callback, this));
  }

private:
  void timer_callback() {
    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel = "map";

    geometry_msgs::msg::TransformStamped t;

    try {
        t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        return;
    }

    tf2::Quaternion q(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pose.set(DubinPoint(t.transform.translation.x, t.transform.translation.y, yaw));
  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
  SafeValue<DubinPoint>& pose;
};

class RoadmapPublisher : public rclcpp::Node
{
  public:
    RoadmapPublisher(RoadMap rm): Node("roadmap_publisher"), count_(0) {
      publisher_ = this->create_publisher<custom_msgs::msg::GeometryGraph>("roadmap", 10);
      std_msgs::msg::Header h;
      h.stamp = this->get_clock()->now();
      auto message = msg_from_roadmap(rm, h);
      publisher_->publish(message);
    }

  private:
    rclcpp::Publisher<custom_msgs::msg::GeometryGraph>::SharedPtr publisher_;
    size_t count_;
};