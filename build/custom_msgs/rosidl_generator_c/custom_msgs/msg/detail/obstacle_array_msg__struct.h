// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/ObstacleArrayMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'obstacles'
#include "custom_msgs/msg/detail/obstacle_msg__struct.h"

/// Struct defined in msg/ObstacleArrayMsg in the package custom_msgs.
/**
  * Message that contains a list of polygon shaped obstacles.
 */
typedef struct custom_msgs__msg__ObstacleArrayMsg
{
  std_msgs__msg__Header header;
  custom_msgs__msg__ObstacleMsg__Sequence obstacles;
} custom_msgs__msg__ObstacleArrayMsg;

// Struct for a sequence of custom_msgs__msg__ObstacleArrayMsg.
typedef struct custom_msgs__msg__ObstacleArrayMsg__Sequence
{
  custom_msgs__msg__ObstacleArrayMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__ObstacleArrayMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__STRUCT_H_
