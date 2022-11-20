#include <chrono>
#include <functional>
#include <memory>
#include <math.h>
#include <cstdlib>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


using namespace std::chrono_literals;

// Implementation of function sinc(t), returning 1 for t==0, and sin(t)/t otherwise
double sinc(double t){
  return abs(t) < 0.002 ?
    1 - (t*t)/6 * (1 - (t*t)/20) :
    sin(t)/t;
}

// Normalize an angle (in range [0,2*pi))
double mod2pi(double angle){
  double out = angle;
  while (out < 0){
    out = out + 2 * M_PI;
  }

  while (out >= 2 * M_PI){
    out = out - 2 * M_PI;
  }

  return out;
}

struct EulerAngles
{
    double pitch, roll, yaw;
};

class DubinPoint{
  public:
    // x coord
    double x;
    // y coord
    double y;
    // orientation (angle between the vector and the x axis) [RAD]
    double th;

    // default constructor
    DubinPoint(void): x{0}, y{0}, th{0} {}

    DubinPoint(double x, double y, double th):x{x}, y{y}, th{th}{}
};

class DubinArc {
  public:
    // starting point
    DubinPoint source;
    // length of the arc
    double length;
    // slope of the segment that joints two successive points
    double k;

    DubinArc(DubinPoint source, double length, double k): source{source}, length{length}, k{k} {}
};

DubinPoint compute_dest(DubinArc arc){
  DubinPoint p;
  p.x = arc.source.x + arc.length * sinc(arc.k * arc.length / 2.0) * cos(arc.source.th + arc.k * arc.length / 2);
  p.y = arc.source.y + arc.length * sinc(arc.k * arc.length / 2.0) * sin(arc.source.th + arc.k * arc.length / 2);
  p.th = mod2pi(arc.source.th + arc.k * arc.length);
  return p;
}

// convert a dubin arc into a set of n_points dubin points.
std::vector<DubinPoint> arc_to_points(DubinArc arc, int n_points){
  std::vector<DubinPoint> points;
  // compute the unit segment
  double unit_seg_length = arc.length / n_points;
  for(int i=0;i<n_points;i++){
    // compute the length of portion of the arc
    double arc_length = unit_seg_length*i;
    // create a smaller dubin arc
    DubinArc new_arc(arc.source, arc_length, arc.k);
    // get the final point of the smaller dubin arc
    DubinPoint p = compute_dest(new_arc);
    // push the final point
    points.push_back(p);
  }
  return points;
}

// convert a vector of dubin points into a vector of poses stamped
std::vector<geometry_msgs::msg::PoseStamped> dubin_points_to_poses(std::vector<DubinPoint> points, builtin_interfaces::msg::Time stamp){
  std::vector<geometry_msgs::msg::PoseStamped> poses;

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = "map";

  // set orientation (same for every pose)
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 0;
  pose.pose.orientation.w = 1;

  for (auto p: points){
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    // set z of point (0 for every pose)
    pose.pose.position.z = 0;
    poses.push_back(pose);
  }

  return poses;
}

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

double to_radians(double angle){
  return 0.0174533*angle;
}

double to_degrees(double angle){
  return angle/0.0174533;
}

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      DubinPoint source(0,0, to_radians(10));
      DubinArc arc(source, 5, to_radians(30));
      auto arc_points = arc_to_points(arc, 20);
      auto arc_poses = dubin_points_to_poses(arc_points, this->get_clock()->now());

      nav_msgs::msg::Path path;
      path.header.stamp = this->get_clock()->now();
      path.header.frame_id = "map";
      path.poses = arc_poses;

      publisher_ = this->create_publisher<nav_msgs::msg::Path>("plan", 10);
      for(auto point:arc_points){
        RCLCPP_INFO(this->get_logger(), "[%f %f]", point.x, point.y);
      }
      publisher_->publish(path);
    }
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}