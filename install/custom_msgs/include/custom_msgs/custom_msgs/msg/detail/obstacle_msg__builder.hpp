// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/ObstacleMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__OBSTACLE_MSG__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__OBSTACLE_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/obstacle_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_ObstacleMsg_radius
{
public:
  explicit Init_ObstacleMsg_radius(::custom_msgs::msg::ObstacleMsg & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::ObstacleMsg radius(::custom_msgs::msg::ObstacleMsg::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::ObstacleMsg msg_;
};

class Init_ObstacleMsg_polygon
{
public:
  explicit Init_ObstacleMsg_polygon(::custom_msgs::msg::ObstacleMsg & msg)
  : msg_(msg)
  {}
  Init_ObstacleMsg_radius polygon(::custom_msgs::msg::ObstacleMsg::_polygon_type arg)
  {
    msg_.polygon = std::move(arg);
    return Init_ObstacleMsg_radius(msg_);
  }

private:
  ::custom_msgs::msg::ObstacleMsg msg_;
};

class Init_ObstacleMsg_header
{
public:
  Init_ObstacleMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObstacleMsg_polygon header(::custom_msgs::msg::ObstacleMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObstacleMsg_polygon(msg_);
  }

private:
  ::custom_msgs::msg::ObstacleMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::ObstacleMsg>()
{
  return custom_msgs::msg::builder::Init_ObstacleMsg_header();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__OBSTACLE_MSG__BUILDER_HPP_
