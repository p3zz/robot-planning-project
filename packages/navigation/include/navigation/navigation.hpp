#include <functional>
#include <memory>
#include <iostream>
#include <thread>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/header.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "graph_msgs/msg/geometry_graph.hpp"
#include "graph_msgs/msg/edges.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "shapes/shapes.hpp"
#include "dubins/dubins.hpp"
#include "map/map.hpp"
#include "planner/planner.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

enum ShelfinoType {
    Pursuer,
    Evader
};

enum ShelfinoStatus {
  Init,
  Idle,
  Moving
};

class EnvironmentDto {
  public:

    EnvironmentDto():gates_position{std::nullopt}, map_borders{std::nullopt}, obstacles{std::nullopt}, roadmap{std::nullopt}{}

    std::optional<std::vector<Point2D>> gates_position;
    std::optional<Polygon> map_borders;
    std::optional<std::vector<Polygon>> obstacles;
    std::optional<RoadMap> roadmap;
};

class ShelfinoDto {
  public:

    ShelfinoDto():pose{std::nullopt}, path_to_follow{std::nullopt}, status{ShelfinoStatus::Init} {}

    std::optional<DubinPoint> pose;
    std::optional<std::vector<DubinCurve>> path_to_follow;
    ShelfinoStatus status;

};

class GatesSubscriber : public rclcpp::Node {
  public:
    GatesSubscriber(std::optional<std::vector<Point2D>>& gates_position);
  private:
    void topic_callback(const geometry_msgs::msg::PoseArray& msg);
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    std::optional<std::vector<Point2D>>& gates_position;
};

class WallsSubscriber : public rclcpp::Node{
  public:
    WallsSubscriber(std::optional<Polygon>& map_borders); 

  private:
    void topic_callback(const geometry_msgs::msg::Polygon & msg);
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_;
    std::optional<Polygon>& map_borders;
};

class ObstaclesSubscriber : public rclcpp::Node{
  public:
    ObstaclesSubscriber(std::optional<std::vector<Polygon>>& obstacles);

  private:
    void topic_callback(const obstacles_msgs::msg::ObstacleArrayMsg & msg);
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_;
    std::optional<std::vector<Polygon>>& obstacles;
};

class PoseSubscriber : public rclcpp::Node{
  public:
      PoseSubscriber(std::optional<DubinPoint>& pose, std::string topic, std::string node_name);

  private:
      void topic_callback(const geometry_msgs::msg::TransformStamped& msg);
      rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
      std::optional<DubinPoint>& pose;
};

class RoadmapPublisher : public rclcpp::Node{
  public:
    RoadmapPublisher(std::optional<RoadMap>& roadmap);

  private:
    rclcpp::Publisher<graph_msgs::msg::GeometryGraph>::SharedPtr publisher_;
    size_t count_;
};

class FollowPathClient : public rclcpp::Node {
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  public:
    FollowPathClient(std::optional<RoadMap>& map, ShelfinoType type, ShelfinoDto& evader, ShelfinoDto& pursuer, std::string service_name, std::string node_name);

  private:
    rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
    std::optional<RoadMap>& map;
    ShelfinoDto& evader;
    ShelfinoDto& pursuer;
    ShelfinoType type;
    rclcpp::TimerBase::SharedPtr timer_;

    bool compute_move();
    void send_goal();
    void result_callback(const GoalHandleFollowPath::WrappedResult& result);
    void timer_callback();
};

class PathPublisher : public rclcpp::Node {
  public:
    PathPublisher(std::optional<std::vector<DubinCurve>>& path, std::string topic_name, std::string node_name);

  private:
    void timer_callback();
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    std::optional<std::vector<DubinCurve>>& path;
    rclcpp::TimerBase::SharedPtr timer_;
};

geometry_msgs::msg::Quaternion to_quaternion(double pitch, double roll, double yaw);
nav_msgs::msg::Path msg_from_curves(std::vector<DubinCurve> curve, std_msgs::msg::Header h);