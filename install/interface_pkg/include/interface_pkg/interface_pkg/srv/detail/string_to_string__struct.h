// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface_pkg:srv/StringToString.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interface_pkg/srv/string_to_string.h"


#ifndef INTERFACE_PKG__SRV__DETAIL__STRING_TO_STRING__STRUCT_H_
#define INTERFACE_PKG__SRV__DETAIL__STRING_TO_STRING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'input'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/StringToString in the package interface_pkg.
typedef struct interface_pkg__srv__StringToString_Request
{
  rosidl_runtime_c__String input;
} interface_pkg__srv__StringToString_Request;

// Struct for a sequence of interface_pkg__srv__StringToString_Request.
typedef struct interface_pkg__srv__StringToString_Request__Sequence
{
  interface_pkg__srv__StringToString_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_pkg__srv__StringToString_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'output'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/StringToString in the package interface_pkg.
typedef struct interface_pkg__srv__StringToString_Response
{
  rosidl_runtime_c__String output;
} interface_pkg__srv__StringToString_Response;

// Struct for a sequence of interface_pkg__srv__StringToString_Response.
typedef struct interface_pkg__srv__StringToString_Response__Sequence
{
  interface_pkg__srv__StringToString_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_pkg__srv__StringToString_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  interface_pkg__srv__StringToString_Event__request__MAX_SIZE = 1
};
// response
enum
{
  interface_pkg__srv__StringToString_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/StringToString in the package interface_pkg.
typedef struct interface_pkg__srv__StringToString_Event
{
  service_msgs__msg__ServiceEventInfo info;
  interface_pkg__srv__StringToString_Request__Sequence request;
  interface_pkg__srv__StringToString_Response__Sequence response;
} interface_pkg__srv__StringToString_Event;

// Struct for a sequence of interface_pkg__srv__StringToString_Event.
typedef struct interface_pkg__srv__StringToString_Event__Sequence
{
  interface_pkg__srv__StringToString_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_pkg__srv__StringToString_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_PKG__SRV__DETAIL__STRING_TO_STRING__STRUCT_H_
