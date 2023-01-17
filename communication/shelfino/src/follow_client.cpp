#include "shelfino/shelfino.hpp"

geometry_msgs::msg::Quaternion to_quaternion(double pitch, double roll, double yaw){
  const double half = 0.5;
  double cr = cos(roll * half);
  double sr = sin(roll * half);
  double cp = cos(pitch * half);
  double sp = sin(pitch * half);
  double cy = cos(yaw * half);
  double sy = sin(yaw * half);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

nav_msgs::msg::Path msg_from_curve(DubinCurve curve, std_msgs::msg::Header h){
  nav_msgs::msg::Path path;
  path.header = h;
  path.header.frame_id = "map";

  geometry_msgs::msg::PoseStamped pose;
  pose.header = h;
  pose.header.frame_id = "map";

  auto trajectory = curve.to_points(50);

  for(auto p:trajectory){
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    pose.pose.position.z = 0;
    pose.pose.orientation = to_quaternion(0,0,p.th);
    path.poses.push_back(pose);
  }

  return path;
}

FollowPathClient::FollowPathClient(std::optional<RoadMap>& map, std::optional<DubinCurve>& path, std::optional<DubinPoint>& evader_pose, std::optional<DubinPoint>& pursuer_pose)
        : Node("follow_path_client"), map{map}, path{path}, evader_pose{evader_pose}, pursuer_pose{pursuer_pose}{
    client_ptr_ = rclcpp_action::create_client<FollowPath>(this, "shelfino2/follow_path");
    this->compute_move();
    this->send_goal();
}

void FollowPathClient::send_goal(){
    RCLCPP_INFO(this->get_logger(), "Waiting for action server");
    if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not found");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Action server found");
    if(!path.has_value()){
        return;
    }

    auto path_msg = FollowPath::Goal();
    std_msgs::msg::Header h;
    h.stamp = this->get_clock()->now();
    path_msg.path = msg_from_curve(path.value(), h);
    path_msg.controller_id = "FollowPath";

    RCLCPP_INFO(this->get_logger(), "Message: ");
    for(auto pose: path_msg.path.poses){
        RCLCPP_INFO(this->get_logger(), "(x: %f, y: %f, z: %f, thx: %f, thy: %f, thz: %f, thw: %f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    }

    auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(&FollowPathClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&FollowPathClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(path_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "Sending goal");
}

void FollowPathClient::goal_response_callback(const GoalHandleFollowPath::SharedPtr& goal_handle){
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void FollowPathClient::feedback_callback(GoalHandleFollowPath::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback){
    RCLCPP_INFO(this->get_logger(), "Speed: %f, Distance to goal: %f", feedback->speed, feedback->distance_to_goal);

    if(feedback->distance_to_goal < 0.5 && !path.has_value()){
        this->compute_move();
    }

    if(feedback->distance_to_goal == 0){
        this->send_goal();
        path.reset();
    }

}

void FollowPathClient::compute_move(){
    if(!map.has_value()){
        RCLCPP_ERROR(this->get_logger(), "Invalid roadmap");
    }
    
    if(!pursuer_pose.has_value() || !evader_pose.has_value()){
        RCLCPP_ERROR(this->get_logger(), "Cannot retrieve shelfino position");
        return;
    }

    PayoffMatrix pm(map.value());
    Path p, e;
    if(!pm.compute_move(pursuer_pose.value(), evader_pose.value(), p, e)){
        RCLCPP_ERROR(this->get_logger(), "No further moves found");
        return;
    }

    path.emplace(p.l1.get_curve());

    RCLCPP_INFO(this->get_logger(), "Path computed");

}

void FollowPathClient::result_callback(const GoalHandleFollowPath::WrappedResult& result){
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }
}
