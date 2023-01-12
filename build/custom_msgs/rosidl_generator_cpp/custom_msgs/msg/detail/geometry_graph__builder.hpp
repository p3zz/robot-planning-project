// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/GeometryGraph.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__GEOMETRY_GRAPH__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__GEOMETRY_GRAPH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/geometry_graph__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_GeometryGraph_edges
{
public:
  explicit Init_GeometryGraph_edges(::custom_msgs::msg::GeometryGraph & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::GeometryGraph edges(::custom_msgs::msg::GeometryGraph::_edges_type arg)
  {
    msg_.edges = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::GeometryGraph msg_;
};

class Init_GeometryGraph_nodes
{
public:
  explicit Init_GeometryGraph_nodes(::custom_msgs::msg::GeometryGraph & msg)
  : msg_(msg)
  {}
  Init_GeometryGraph_edges nodes(::custom_msgs::msg::GeometryGraph::_nodes_type arg)
  {
    msg_.nodes = std::move(arg);
    return Init_GeometryGraph_edges(msg_);
  }

private:
  ::custom_msgs::msg::GeometryGraph msg_;
};

class Init_GeometryGraph_header
{
public:
  Init_GeometryGraph_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GeometryGraph_nodes header(::custom_msgs::msg::GeometryGraph::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GeometryGraph_nodes(msg_);
  }

private:
  ::custom_msgs::msg::GeometryGraph msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::GeometryGraph>()
{
  return custom_msgs::msg::builder::Init_GeometryGraph_header();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__GEOMETRY_GRAPH__BUILDER_HPP_
