// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from yolo_depth_interfaces:msg/ObjectDepth.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "yolo_depth_interfaces/msg/detail/object_depth__rosidl_typesupport_introspection_c.h"
#include "yolo_depth_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "yolo_depth_interfaces/msg/detail/object_depth__functions.h"
#include "yolo_depth_interfaces/msg/detail/object_depth__struct.h"


// Include directives for member types
// Member `class_name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void yolo_depth_interfaces__msg__ObjectDepth__rosidl_typesupport_introspection_c__ObjectDepth_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yolo_depth_interfaces__msg__ObjectDepth__init(message_memory);
}

void yolo_depth_interfaces__msg__ObjectDepth__rosidl_typesupport_introspection_c__ObjectDepth_fini_function(void * message_memory)
{
  yolo_depth_interfaces__msg__ObjectDepth__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember yolo_depth_interfaces__msg__ObjectDepth__rosidl_typesupport_introspection_c__ObjectDepth_message_member_array[7] = {
  {
    "class_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_depth_interfaces__msg__ObjectDepth, class_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_depth_interfaces__msg__ObjectDepth, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance_m",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_depth_interfaces__msg__ObjectDepth, distance_m),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_depth_interfaces__msg__ObjectDepth, x1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_depth_interfaces__msg__ObjectDepth, y1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_depth_interfaces__msg__ObjectDepth, x2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_depth_interfaces__msg__ObjectDepth, y2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers yolo_depth_interfaces__msg__ObjectDepth__rosidl_typesupport_introspection_c__ObjectDepth_message_members = {
  "yolo_depth_interfaces__msg",  // message namespace
  "ObjectDepth",  // message name
  7,  // number of fields
  sizeof(yolo_depth_interfaces__msg__ObjectDepth),
  false,  // has_any_key_member_
  yolo_depth_interfaces__msg__ObjectDepth__rosidl_typesupport_introspection_c__ObjectDepth_message_member_array,  // message members
  yolo_depth_interfaces__msg__ObjectDepth__rosidl_typesupport_introspection_c__ObjectDepth_init_function,  // function to initialize message memory (memory has to be allocated)
  yolo_depth_interfaces__msg__ObjectDepth__rosidl_typesupport_introspection_c__ObjectDepth_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t yolo_depth_interfaces__msg__ObjectDepth__rosidl_typesupport_introspection_c__ObjectDepth_message_type_support_handle = {
  0,
  &yolo_depth_interfaces__msg__ObjectDepth__rosidl_typesupport_introspection_c__ObjectDepth_message_members,
  get_message_typesupport_handle_function,
  &yolo_depth_interfaces__msg__ObjectDepth__get_type_hash,
  &yolo_depth_interfaces__msg__ObjectDepth__get_type_description,
  &yolo_depth_interfaces__msg__ObjectDepth__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolo_depth_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_depth_interfaces, msg, ObjectDepth)() {
  if (!yolo_depth_interfaces__msg__ObjectDepth__rosidl_typesupport_introspection_c__ObjectDepth_message_type_support_handle.typesupport_identifier) {
    yolo_depth_interfaces__msg__ObjectDepth__rosidl_typesupport_introspection_c__ObjectDepth_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &yolo_depth_interfaces__msg__ObjectDepth__rosidl_typesupport_introspection_c__ObjectDepth_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
