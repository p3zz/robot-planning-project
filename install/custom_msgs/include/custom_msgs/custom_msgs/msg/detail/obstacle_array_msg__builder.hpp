// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/ObstacleArrayMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/obstacle_array_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_ObstacleArrayMsg_obstacles
{
public:
  explicit Init_ObstacleArrayMsg_obstacles(::custom_msgs::msg::ObstacleArrayMsg & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::ObstacleArrayMsg obstacles(::custom_msgs::msg::ObstacleArrayMsg::_obstacles_type arg)
  {
    msg_.obstacles = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::ObstacleArrayMsg msg_;
};

class Init_ObstacleArrayMsg_header
{
public:
  Init_ObstacleArrayMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObstacleArrayMsg_obstacles header(::custom_msgs::msg::ObstacleArrayMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObstacleArrayMsg_obstacles(msg_);
  }

private:
  ::custom_msgs::msg::ObstacleArrayMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::ObstacleArrayMsg>()
{
  return custom_msgs::msg::builder::Init_ObstacleArrayMsg_header();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__BUILDER_HPP_
