// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolo_depth_interfaces:msg/ObjectDepth.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "yolo_depth_interfaces/msg/object_depth.h"


#ifndef YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH__STRUCT_H_
#define YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'class_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ObjectDepth in the package yolo_depth_interfaces.
typedef struct yolo_depth_interfaces__msg__ObjectDepth
{
  rosidl_runtime_c__String class_name;
  float confidence;
  float distance_m;
  int32_t x1;
  int32_t y1;
  int32_t x2;
  int32_t y2;
} yolo_depth_interfaces__msg__ObjectDepth;

// Struct for a sequence of yolo_depth_interfaces__msg__ObjectDepth.
typedef struct yolo_depth_interfaces__msg__ObjectDepth__Sequence
{
  yolo_depth_interfaces__msg__ObjectDepth * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo_depth_interfaces__msg__ObjectDepth__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH__STRUCT_H_
