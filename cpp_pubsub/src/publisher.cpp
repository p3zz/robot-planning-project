#include <chrono>
#include <functional>
#include <memory>
#include <math.h>
#include <cstdlib>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lib_dubins/dubins.h"
#include "path_solver/path_solver.h"

using namespace std;

struct EulerAngles
{
    double pitch, roll, yaw;
};

geometry_msgs::msg::Quaternion to_quaternion(EulerAngles angles) // roll (x), pitch (Y), yaw (z)
{
  const double half = 0.5;
  double cr = cos(angles.roll * half);
  double sr = sin(angles.roll * half);
  double cp = cos(angles.pitch * half);
  double sp = sin(angles.pitch * half);
  double cy = cos(angles.yaw * half);
  double sy = sin(angles.yaw * half);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

EulerAngles to_euler_angles(geometry_msgs::msg::Quaternion q){
  EulerAngles angles;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  angles.roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q.w * q.y - q.z * q.x);
  if (std::abs(sinp) >= 1)
      angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
      angles.pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  angles.yaw = std::atan2(siny_cosp, cosy_cosp);

  return angles;
}

// convert a vector of dubin points into a vector of poses stamped
std::vector<geometry_msgs::msg::PoseStamped> dubin_points_to_poses(DubinPoint start, DubinPoint end, builtin_interfaces::msg::Time stamp){
  std::vector<geometry_msgs::msg::PoseStamped> poses;

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = "map";

  //dubins
  DubinCurve curve = dubins_shortest_path(start, end);
  vector<Point> trajectory = curve.get_trajectory();

  for (Point p:trajectory){
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    pose.pose.position.z = 0;
    EulerAngles ea;
    ea.roll=0;
    ea.pitch=0;
    ea.yaw=p.th;
    pose.pose.orientation = to_quaternion(ea);
    poses.push_back(pose);
  }

  return poses;
}

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      DubinPoint p_start(0.0, 0.0, M_PI/4);
      DubinPoint p_end(2.0, 3.0, 0);
      auto arc_poses = dubin_points_to_poses(p_start, p_end, this->get_clock()->now());

      nav_msgs::msg::Path path;
      path.header.stamp = this->get_clock()->now();
      path.header.frame_id = "map";
      path.poses = arc_poses;

      publisher_ = this->create_publisher<nav_msgs::msg::Path>("plan", 10);
      //keep channel up for 10 times (100ms)
      for (size_t i = 0; i < 10; i++)
      {
        publisher_->publish(path);
        this_thread::sleep_for(chrono::milliseconds(10));
      }
    }
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    size_t count_;
};

void randomObstacles(Room* room, int num_obstacles, const int max_side)
{
  Point2D* centers = new Point2D[num_obstacles];
  for(int i=0; i<num_obstacles; i++)
  {
    Obstacle o;
    bool check;
    do{
      centers[i].x = ((rand()%(room->getWidth()*100-max_side))+max_side/2)/100.0;
      centers[i].y = ((rand()%(room->getHeight()*100-max_side))+max_side/2)/100.0;
      check=true;
      for(int j=0;j<i;j++)
        if(sqrt(pow(centers[j].x-centers[i].x,2)+pow(centers[j].y-centers[i].y,2))<max_side/100.0*sqrt(2))
        {
          check=false;
          break;
        }
    }while(!check);
    Point2D center(centers[i]);
    o.addVertex(Point2D(center.x+((rand() %(max_side/4))-(max_side/2))/100.0,center.y+((rand() %(max_side/4))+(max_side/4))/100.0));
    o.addVertex(Point2D(center.x+((rand() %(max_side/4))+(max_side/4))/100.0,center.y+((rand() %(max_side/4))+(max_side/4))/100.0));
    o.addVertex(Point2D(center.x+((rand() %(max_side/4))+(max_side/4))/100.0,center.y+((rand() %(max_side/4))-(max_side/2))/100.0));
    o.addVertex(Point2D(center.x+((rand() %(max_side/4))-(max_side/2))/100.0,center.y+((rand() %(max_side/4))-(max_side/2))/100.0));
    room->addObstacle(o);
  }
}

int main(/*int argc, char * argv[]*/)
{  
  //rclcpp::init(argc, argv);
  //rclcpp::spin(std::make_shared<MinimalPublisher>());
  //rclcpp::shutdown();
  srand(time(NULL));
  Room room(10,10);
  randomObstacles(&room, 4, 200);
  RoadMap map(room);
  if(map.constructRoadMap(20, 4, 0.5, 500)) //knn=4 is the best choice (up, down, left and right in the ideal case)
    cout<<map.getJson()<<endl;
  else
    cout<<"Error k"<<endl;
  return 0;
}
