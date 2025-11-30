// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from yolo_depth_interfaces:srv/SetTarget.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "yolo_depth_interfaces/srv/detail/set_target__rosidl_typesupport_introspection_c.h"
#include "yolo_depth_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "yolo_depth_interfaces/srv/detail/set_target__functions.h"
#include "yolo_depth_interfaces/srv/detail/set_target__struct.h"


// Include directives for member types
// Member `target`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void yolo_depth_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yolo_depth_interfaces__srv__SetTarget_Request__init(message_memory);
}

void yolo_depth_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_fini_function(void * message_memory)
{
  yolo_depth_interfaces__srv__SetTarget_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember yolo_depth_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_member_array[1] = {
  {
    "target",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_depth_interfaces__srv__SetTarget_Request, target),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers yolo_depth_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_members = {
  "yolo_depth_interfaces__srv",  // message namespace
  "SetTarget_Request",  // message name
  1,  // number of fields
  sizeof(yolo_depth_interfaces__srv__SetTarget_Request),
  false,  // has_any_key_member_
  yolo_depth_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_member_array,  // message members
  yolo_depth_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  yolo_depth_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t yolo_depth_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_type_support_handle = {
  0,
  &yolo_depth_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_members,
  get_message_typesupport_handle_function,
  &yolo_depth_interfaces__srv__SetTarget_Request__get_type_hash,
  &yolo_depth_interfaces__srv__SetTarget_Request__get_type_description,
  &yolo_depth_interfaces__srv__SetTarget_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolo_depth_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_depth_interfaces, srv, SetTarget_Request)() {
  if (!yolo_depth_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_type_support_handle.typesupport_identifier) {
    yolo_depth_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &yolo_depth_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "yolo_depth_interfaces/srv/detail/set_target__rosidl_typesupport_introspection_c.h"
// already included above
// #include "yolo_depth_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "yolo_depth_interfaces/srv/detail/set_target__functions.h"
// already included above
// #include "yolo_depth_interfaces/srv/detail/set_target__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void yolo_depth_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yolo_depth_interfaces__srv__SetTarget_Response__init(message_memory);
}

void yolo_depth_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_fini_function(void * message_memory)
{
  yolo_depth_interfaces__srv__SetTarget_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember yolo_depth_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_depth_interfaces__srv__SetTarget_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_depth_interfaces__srv__SetTarget_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers yolo_depth_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_members = {
  "yolo_depth_interfaces__srv",  // message namespace
  "SetTarget_Response",  // message name
  2,  // number of fields
  sizeof(yolo_depth_interfaces__srv__SetTarget_Response),
  false,  // has_any_key_member_
  yolo_depth_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_member_array,  // message members
  yolo_depth_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  yolo_depth_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t yolo_depth_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_type_support_handle = {
  0,
  &yolo_depth_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_members,
  get_message_typesupport_handle_function,
  &yolo_depth_interfaces__srv__SetTarget_Response__get_type_hash,
  &yolo_depth_interfaces__srv__SetTarget_Response__get_type_description,
  &yolo_depth_interfaces__srv__SetTarget_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolo_depth_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_depth_interfaces, srv, SetTarget_Response)() {
  if (!yolo_depth_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_type_support_handle.typesupport_identifier) {
    yolo_depth_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &yolo_depth_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "yolo_depth_interfaces/srv/detail/set_target__rosidl_typesupport_introspection_c.h"
// already included above
// #include "yolo_depth_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "yolo_depth_interfaces/srv/detail/set_target__functions.h"
// already included above
// #include "yolo_depth_interfaces/srv/detail/set_target__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "yolo_depth_interfaces/srv/set_target.h"
// Member `request`
// Member `response`
// already included above
// #include "yolo_depth_interfaces/srv/detail/set_target__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__SetTarget_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yolo_depth_interfaces__srv__SetTarget_Event__init(message_memory);
}

void yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__SetTarget_Event_fini_function(void * message_memory)
{
  yolo_depth_interfaces__srv__SetTarget_Event__fini(message_memory);
}

size_t yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__size_function__SetTarget_Event__request(
  const void * untyped_member)
{
  const yolo_depth_interfaces__srv__SetTarget_Request__Sequence * member =
    (const yolo_depth_interfaces__srv__SetTarget_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__get_const_function__SetTarget_Event__request(
  const void * untyped_member, size_t index)
{
  const yolo_depth_interfaces__srv__SetTarget_Request__Sequence * member =
    (const yolo_depth_interfaces__srv__SetTarget_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__get_function__SetTarget_Event__request(
  void * untyped_member, size_t index)
{
  yolo_depth_interfaces__srv__SetTarget_Request__Sequence * member =
    (yolo_depth_interfaces__srv__SetTarget_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__fetch_function__SetTarget_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const yolo_depth_interfaces__srv__SetTarget_Request * item =
    ((const yolo_depth_interfaces__srv__SetTarget_Request *)
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__get_const_function__SetTarget_Event__request(untyped_member, index));
  yolo_depth_interfaces__srv__SetTarget_Request * value =
    (yolo_depth_interfaces__srv__SetTarget_Request *)(untyped_value);
  *value = *item;
}

void yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__assign_function__SetTarget_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  yolo_depth_interfaces__srv__SetTarget_Request * item =
    ((yolo_depth_interfaces__srv__SetTarget_Request *)
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__get_function__SetTarget_Event__request(untyped_member, index));
  const yolo_depth_interfaces__srv__SetTarget_Request * value =
    (const yolo_depth_interfaces__srv__SetTarget_Request *)(untyped_value);
  *item = *value;
}

bool yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__resize_function__SetTarget_Event__request(
  void * untyped_member, size_t size)
{
  yolo_depth_interfaces__srv__SetTarget_Request__Sequence * member =
    (yolo_depth_interfaces__srv__SetTarget_Request__Sequence *)(untyped_member);
  yolo_depth_interfaces__srv__SetTarget_Request__Sequence__fini(member);
  return yolo_depth_interfaces__srv__SetTarget_Request__Sequence__init(member, size);
}

size_t yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__size_function__SetTarget_Event__response(
  const void * untyped_member)
{
  const yolo_depth_interfaces__srv__SetTarget_Response__Sequence * member =
    (const yolo_depth_interfaces__srv__SetTarget_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__get_const_function__SetTarget_Event__response(
  const void * untyped_member, size_t index)
{
  const yolo_depth_interfaces__srv__SetTarget_Response__Sequence * member =
    (const yolo_depth_interfaces__srv__SetTarget_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__get_function__SetTarget_Event__response(
  void * untyped_member, size_t index)
{
  yolo_depth_interfaces__srv__SetTarget_Response__Sequence * member =
    (yolo_depth_interfaces__srv__SetTarget_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__fetch_function__SetTarget_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const yolo_depth_interfaces__srv__SetTarget_Response * item =
    ((const yolo_depth_interfaces__srv__SetTarget_Response *)
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__get_const_function__SetTarget_Event__response(untyped_member, index));
  yolo_depth_interfaces__srv__SetTarget_Response * value =
    (yolo_depth_interfaces__srv__SetTarget_Response *)(untyped_value);
  *value = *item;
}

void yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__assign_function__SetTarget_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  yolo_depth_interfaces__srv__SetTarget_Response * item =
    ((yolo_depth_interfaces__srv__SetTarget_Response *)
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__get_function__SetTarget_Event__response(untyped_member, index));
  const yolo_depth_interfaces__srv__SetTarget_Response * value =
    (const yolo_depth_interfaces__srv__SetTarget_Response *)(untyped_value);
  *item = *value;
}

bool yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__resize_function__SetTarget_Event__response(
  void * untyped_member, size_t size)
{
  yolo_depth_interfaces__srv__SetTarget_Response__Sequence * member =
    (yolo_depth_interfaces__srv__SetTarget_Response__Sequence *)(untyped_member);
  yolo_depth_interfaces__srv__SetTarget_Response__Sequence__fini(member);
  return yolo_depth_interfaces__srv__SetTarget_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__SetTarget_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_depth_interfaces__srv__SetTarget_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(yolo_depth_interfaces__srv__SetTarget_Event, request),  // bytes offset in struct
    NULL,  // default value
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__size_function__SetTarget_Event__request,  // size() function pointer
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__get_const_function__SetTarget_Event__request,  // get_const(index) function pointer
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__get_function__SetTarget_Event__request,  // get(index) function pointer
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__fetch_function__SetTarget_Event__request,  // fetch(index, &value) function pointer
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__assign_function__SetTarget_Event__request,  // assign(index, value) function pointer
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__resize_function__SetTarget_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(yolo_depth_interfaces__srv__SetTarget_Event, response),  // bytes offset in struct
    NULL,  // default value
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__size_function__SetTarget_Event__response,  // size() function pointer
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__get_const_function__SetTarget_Event__response,  // get_const(index) function pointer
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__get_function__SetTarget_Event__response,  // get(index) function pointer
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__fetch_function__SetTarget_Event__response,  // fetch(index, &value) function pointer
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__assign_function__SetTarget_Event__response,  // assign(index, value) function pointer
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__resize_function__SetTarget_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__SetTarget_Event_message_members = {
  "yolo_depth_interfaces__srv",  // message namespace
  "SetTarget_Event",  // message name
  3,  // number of fields
  sizeof(yolo_depth_interfaces__srv__SetTarget_Event),
  false,  // has_any_key_member_
  yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__SetTarget_Event_message_member_array,  // message members
  yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__SetTarget_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__SetTarget_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__SetTarget_Event_message_type_support_handle = {
  0,
  &yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__SetTarget_Event_message_members,
  get_message_typesupport_handle_function,
  &yolo_depth_interfaces__srv__SetTarget_Event__get_type_hash,
  &yolo_depth_interfaces__srv__SetTarget_Event__get_type_description,
  &yolo_depth_interfaces__srv__SetTarget_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolo_depth_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_depth_interfaces, srv, SetTarget_Event)() {
  yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__SetTarget_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__SetTarget_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_depth_interfaces, srv, SetTarget_Request)();
  yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__SetTarget_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_depth_interfaces, srv, SetTarget_Response)();
  if (!yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__SetTarget_Event_message_type_support_handle.typesupport_identifier) {
    yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__SetTarget_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__SetTarget_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "yolo_depth_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "yolo_depth_interfaces/srv/detail/set_target__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers yolo_depth_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_service_members = {
  "yolo_depth_interfaces__srv",  // service namespace
  "SetTarget",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // yolo_depth_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_Request_message_type_support_handle,
  NULL,  // response message
  // yolo_depth_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_Response_message_type_support_handle
  NULL  // event_message
  // yolo_depth_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_Response_message_type_support_handle
};


static rosidl_service_type_support_t yolo_depth_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_service_type_support_handle = {
  0,
  &yolo_depth_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_service_members,
  get_service_typesupport_handle_function,
  &yolo_depth_interfaces__srv__SetTarget_Request__rosidl_typesupport_introspection_c__SetTarget_Request_message_type_support_handle,
  &yolo_depth_interfaces__srv__SetTarget_Response__rosidl_typesupport_introspection_c__SetTarget_Response_message_type_support_handle,
  &yolo_depth_interfaces__srv__SetTarget_Event__rosidl_typesupport_introspection_c__SetTarget_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    yolo_depth_interfaces,
    srv,
    SetTarget
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    yolo_depth_interfaces,
    srv,
    SetTarget
  ),
  &yolo_depth_interfaces__srv__SetTarget__get_type_hash,
  &yolo_depth_interfaces__srv__SetTarget__get_type_description,
  &yolo_depth_interfaces__srv__SetTarget__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_depth_interfaces, srv, SetTarget_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_depth_interfaces, srv, SetTarget_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_depth_interfaces, srv, SetTarget_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolo_depth_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_depth_interfaces, srv, SetTarget)(void) {
  if (!yolo_depth_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_service_type_support_handle.typesupport_identifier) {
    yolo_depth_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)yolo_depth_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_depth_interfaces, srv, SetTarget_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_depth_interfaces, srv, SetTarget_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_depth_interfaces, srv, SetTarget_Event)()->data;
  }

  return &yolo_depth_interfaces__srv__detail__set_target__rosidl_typesupport_introspection_c__SetTarget_service_type_support_handle;
}
