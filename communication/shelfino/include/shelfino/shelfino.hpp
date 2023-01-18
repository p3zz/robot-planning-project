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
#include "decisions/decisions.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class ShelfinoDto {
    public:
        ShelfinoDto():gates_position{std::nullopt}, map_borders{std::nullopt}, obstacles{std::nullopt},
          pursuer_pose{std::nullopt}, evader_pose{std::nullopt}, roadmap{std::nullopt}, path_to_follow{std::nullopt}{}

        std::optional<std::vector<Point2D>> gates_position;
        std::optional<Polygon> map_borders;
        std::optional<std::vector<Polygon>> obstacles;
        std::optional<DubinPoint> pursuer_pose;
        std::optional<DubinPoint> evader_pose;
        std::optional<RoadMap> roadmap;
        std::optional<DubinCurve> path_to_follow;
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
    FollowPathClient(std::optional<RoadMap>& map, std::optional<DubinCurve>& path, std::optional<DubinPoint>& evader_pose, std::optional<DubinPoint>& pursuer_pose);

  private:
    rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
    std::optional<DubinCurve>& path;
    std::optional<RoadMap>& map;
    std::optional<DubinPoint>& evader_pose;
    std::optional<DubinPoint>& pursuer_pose;

    void compute_move();
    void send_goal();
    void goal_response_callback(const GoalHandleFollowPath::SharedPtr& goal_handle);
    void feedback_callback(GoalHandleFollowPath::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback);
    void result_callback(const GoalHandleFollowPath::WrappedResult& result);
};

class PathPublisher : public rclcpp::Node {
  public:
    PathPublisher(std::optional<DubinCurve>& path);

  private:
    void timer_callback();
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    std::optional<DubinCurve>& path;
    rclcpp::TimerBase::SharedPtr timer_;
};

geometry_msgs::msg::Quaternion to_quaternion(double pitch, double roll, double yaw);
nav_msgs::msg::Path msg_from_curve(DubinCurve curve, std_msgs::msg::Header h);