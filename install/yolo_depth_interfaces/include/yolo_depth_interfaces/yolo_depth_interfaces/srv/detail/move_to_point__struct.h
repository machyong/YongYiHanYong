// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolo_depth_interfaces:srv/MoveToPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "yolo_depth_interfaces/srv/move_to_point.h"


#ifndef YOLO_DEPTH_INTERFACES__SRV__DETAIL__MOVE_TO_POINT__STRUCT_H_
#define YOLO_DEPTH_INTERFACES__SRV__DETAIL__MOVE_TO_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_position'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in srv/MoveToPoint in the package yolo_depth_interfaces.
typedef struct yolo_depth_interfaces__srv__MoveToPoint_Request
{
  geometry_msgs__msg__Point target_position;
} yolo_depth_interfaces__srv__MoveToPoint_Request;

// Struct for a sequence of yolo_depth_interfaces__srv__MoveToPoint_Request.
typedef struct yolo_depth_interfaces__srv__MoveToPoint_Request__Sequence
{
  yolo_depth_interfaces__srv__MoveToPoint_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo_depth_interfaces__srv__MoveToPoint_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MoveToPoint in the package yolo_depth_interfaces.
typedef struct yolo_depth_interfaces__srv__MoveToPoint_Response
{
  bool success;
  rosidl_runtime_c__String message;
} yolo_depth_interfaces__srv__MoveToPoint_Response;

// Struct for a sequence of yolo_depth_interfaces__srv__MoveToPoint_Response.
typedef struct yolo_depth_interfaces__srv__MoveToPoint_Response__Sequence
{
  yolo_depth_interfaces__srv__MoveToPoint_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo_depth_interfaces__srv__MoveToPoint_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  yolo_depth_interfaces__srv__MoveToPoint_Event__request__MAX_SIZE = 1
};
// response
enum
{
  yolo_depth_interfaces__srv__MoveToPoint_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/MoveToPoint in the package yolo_depth_interfaces.
typedef struct yolo_depth_interfaces__srv__MoveToPoint_Event
{
  service_msgs__msg__ServiceEventInfo info;
  yolo_depth_interfaces__srv__MoveToPoint_Request__Sequence request;
  yolo_depth_interfaces__srv__MoveToPoint_Response__Sequence response;
} yolo_depth_interfaces__srv__MoveToPoint_Event;

// Struct for a sequence of yolo_depth_interfaces__srv__MoveToPoint_Event.
typedef struct yolo_depth_interfaces__srv__MoveToPoint_Event__Sequence
{
  yolo_depth_interfaces__srv__MoveToPoint_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo_depth_interfaces__srv__MoveToPoint_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLO_DEPTH_INTERFACES__SRV__DETAIL__MOVE_TO_POINT__STRUCT_H_
