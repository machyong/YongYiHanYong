// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolo_depth_interfaces:srv/TargetCenter.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "yolo_depth_interfaces/srv/target_center.h"


#ifndef YOLO_DEPTH_INTERFACES__SRV__DETAIL__TARGET_CENTER__STRUCT_H_
#define YOLO_DEPTH_INTERFACES__SRV__DETAIL__TARGET_CENTER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/TargetCenter in the package yolo_depth_interfaces.
typedef struct yolo_depth_interfaces__srv__TargetCenter_Request
{
  rosidl_runtime_c__String target;
  double center_x;
  double center_y;
  double distance_m;
} yolo_depth_interfaces__srv__TargetCenter_Request;

// Struct for a sequence of yolo_depth_interfaces__srv__TargetCenter_Request.
typedef struct yolo_depth_interfaces__srv__TargetCenter_Request__Sequence
{
  yolo_depth_interfaces__srv__TargetCenter_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo_depth_interfaces__srv__TargetCenter_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/TargetCenter in the package yolo_depth_interfaces.
typedef struct yolo_depth_interfaces__srv__TargetCenter_Response
{
  bool success;
  rosidl_runtime_c__String message;
} yolo_depth_interfaces__srv__TargetCenter_Response;

// Struct for a sequence of yolo_depth_interfaces__srv__TargetCenter_Response.
typedef struct yolo_depth_interfaces__srv__TargetCenter_Response__Sequence
{
  yolo_depth_interfaces__srv__TargetCenter_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo_depth_interfaces__srv__TargetCenter_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  yolo_depth_interfaces__srv__TargetCenter_Event__request__MAX_SIZE = 1
};
// response
enum
{
  yolo_depth_interfaces__srv__TargetCenter_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/TargetCenter in the package yolo_depth_interfaces.
typedef struct yolo_depth_interfaces__srv__TargetCenter_Event
{
  service_msgs__msg__ServiceEventInfo info;
  yolo_depth_interfaces__srv__TargetCenter_Request__Sequence request;
  yolo_depth_interfaces__srv__TargetCenter_Response__Sequence response;
} yolo_depth_interfaces__srv__TargetCenter_Event;

// Struct for a sequence of yolo_depth_interfaces__srv__TargetCenter_Event.
typedef struct yolo_depth_interfaces__srv__TargetCenter_Event__Sequence
{
  yolo_depth_interfaces__srv__TargetCenter_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo_depth_interfaces__srv__TargetCenter_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLO_DEPTH_INTERFACES__SRV__DETAIL__TARGET_CENTER__STRUCT_H_
