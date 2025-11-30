// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolo_depth_interfaces:msg/ObjectDepthArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "yolo_depth_interfaces/msg/object_depth_array.h"


#ifndef YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH_ARRAY__STRUCT_H_
#define YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'objects'
#include "yolo_depth_interfaces/msg/detail/object_depth__struct.h"

/// Struct defined in msg/ObjectDepthArray in the package yolo_depth_interfaces.
typedef struct yolo_depth_interfaces__msg__ObjectDepthArray
{
  yolo_depth_interfaces__msg__ObjectDepth__Sequence objects;
} yolo_depth_interfaces__msg__ObjectDepthArray;

// Struct for a sequence of yolo_depth_interfaces__msg__ObjectDepthArray.
typedef struct yolo_depth_interfaces__msg__ObjectDepthArray__Sequence
{
  yolo_depth_interfaces__msg__ObjectDepthArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo_depth_interfaces__msg__ObjectDepthArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH_ARRAY__STRUCT_H_
