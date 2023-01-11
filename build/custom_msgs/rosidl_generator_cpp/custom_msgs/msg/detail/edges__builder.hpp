// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/Edges.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__EDGES__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__EDGES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/edges__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_Edges_weights
{
public:
  explicit Init_Edges_weights(::custom_msgs::msg::Edges & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::Edges weights(::custom_msgs::msg::Edges::_weights_type arg)
  {
    msg_.weights = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::Edges msg_;
};

class Init_Edges_node_ids
{
public:
  Init_Edges_node_ids()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Edges_weights node_ids(::custom_msgs::msg::Edges::_node_ids_type arg)
  {
    msg_.node_ids = std::move(arg);
    return Init_Edges_weights(msg_);
  }

private:
  ::custom_msgs::msg::Edges msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::Edges>()
{
  return custom_msgs::msg::builder::Init_Edges_node_ids();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__EDGES__BUILDER_HPP_
