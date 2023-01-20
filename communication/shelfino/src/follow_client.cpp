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

FollowPathClient::FollowPathClient(std::optional<RoadMap>& map, ShelfinoType type, ShelfinoDto& evader, ShelfinoDto& pursuer,
    std::string service_name, std::string node_name) : Node(node_name), map{map}, evader{evader}, pursuer{pursuer}, type{type}{
    client_ptr_ = rclcpp_action::create_client<FollowPath>(this, service_name);
    this->compute_move();
    this->send_goal();
}

void FollowPathClient::send_goal(){
    RCLCPP_INFO(this->get_logger(), "Waiting for action server");
    if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not found");
        exit(2);
    }
    RCLCPP_INFO(this->get_logger(), "Action server found");

    auto path_msg = FollowPath::Goal();
    std_msgs::msg::Header h;
    h.stamp = this->get_clock()->now();

    if(type == ShelfinoType::Pursuer){
        path_msg.path = msg_from_curve(pursuer.path_to_follow.value(), h);
    }else{
        path_msg.path = msg_from_curve(evader.path_to_follow.value(), h);
    }

    path_msg.controller_id = "FollowPath";

    RCLCPP_INFO(this->get_logger(), "Message: ");
    for(auto pose: path_msg.path.poses){
        RCLCPP_INFO(this->get_logger(), "(x: %f, y: %f, z: %f, thx: %f, thy: %f, thz: %f, thw: %f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    }

    auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&FollowPathClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(path_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "Sending goal");
}

bool FollowPathClient::compute_move(){
    // check if the map exists
    if(!map.has_value()){
        RCLCPP_ERROR(this->get_logger(), "Invalid roadmap");
        return false;
    }
    
    // check if both pursuer and evader poses exist
    if(!pursuer.pose.has_value() || !evader.pose.has_value()){
        RCLCPP_ERROR(this->get_logger(), "Cannot retrieve shelfino position");
        return false;
    }

    // wait until both shelfinos are idle (not moving)
    while(evader.status == ShelfinoStatus::Moving || pursuer.status == ShelfinoStatus::Moving){}

    PayoffMatrix pm(map.value());
    Path p, e;
    // if there are no moves found for both shelfinos, exit
    if(!pm.compute_move(pursuer.pose.value(), evader.pose.value(), p, e)){
        RCLCPP_ERROR(this->get_logger(), "No further moves found");
        exit(2);
    }

    // if pursuer and evader have the same destination or the destination of the evader is the start of the pursuer, catch!
    if(e.l1.get_dst() == p.l1.get_dst() || e.l1.get_dst() == p.l1.get_src()){
        RCLCPP_INFO(this->get_logger(), "Pursuer reaches the evader");
        exit(1);
    }

    // if the evader reaches the destination, :(
    auto room = map.value().get_room();
    for(int i=0;i<room.get_num_exits();i++){
        Point2D dst(e.l1.get_dst().x, e.l1.get_dst().y);
        if(dst == room.get_exit(i, true)){
            RCLCPP_ERROR(this->get_logger(), "Evader reaches the gate");
            exit(2);
        }
    }

    pursuer.path_to_follow.emplace(p.l1.get_curve());
    evader.path_to_follow.emplace(e.l1.get_curve());

    RCLCPP_INFO(this->get_logger(), "Path computed");

    return true;
}

void FollowPathClient::result_callback(const GoalHandleFollowPath::WrappedResult& result){
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
        case rclcpp_action::ResultCode::ABORTED:
            if(this->compute_move()){
                this->send_goal();
            }
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
