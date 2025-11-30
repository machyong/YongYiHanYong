// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from interface_pkg:srv/StringToString.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "interface_pkg/srv/detail/string_to_string__rosidl_typesupport_introspection_c.h"
#include "interface_pkg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "interface_pkg/srv/detail/string_to_string__functions.h"
#include "interface_pkg/srv/detail/string_to_string__struct.h"


// Include directives for member types
// Member `input`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void interface_pkg__srv__StringToString_Request__rosidl_typesupport_introspection_c__StringToString_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interface_pkg__srv__StringToString_Request__init(message_memory);
}

void interface_pkg__srv__StringToString_Request__rosidl_typesupport_introspection_c__StringToString_Request_fini_function(void * message_memory)
{
  interface_pkg__srv__StringToString_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember interface_pkg__srv__StringToString_Request__rosidl_typesupport_introspection_c__StringToString_Request_message_member_array[1] = {
  {
    "input",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_pkg__srv__StringToString_Request, input),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers interface_pkg__srv__StringToString_Request__rosidl_typesupport_introspection_c__StringToString_Request_message_members = {
  "interface_pkg__srv",  // message namespace
  "StringToString_Request",  // message name
  1,  // number of fields
  sizeof(interface_pkg__srv__StringToString_Request),
  false,  // has_any_key_member_
  interface_pkg__srv__StringToString_Request__rosidl_typesupport_introspection_c__StringToString_Request_message_member_array,  // message members
  interface_pkg__srv__StringToString_Request__rosidl_typesupport_introspection_c__StringToString_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  interface_pkg__srv__StringToString_Request__rosidl_typesupport_introspection_c__StringToString_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t interface_pkg__srv__StringToString_Request__rosidl_typesupport_introspection_c__StringToString_Request_message_type_support_handle = {
  0,
  &interface_pkg__srv__StringToString_Request__rosidl_typesupport_introspection_c__StringToString_Request_message_members,
  get_message_typesupport_handle_function,
  &interface_pkg__srv__StringToString_Request__get_type_hash,
  &interface_pkg__srv__StringToString_Request__get_type_description,
  &interface_pkg__srv__StringToString_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface_pkg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_pkg, srv, StringToString_Request)() {
  if (!interface_pkg__srv__StringToString_Request__rosidl_typesupport_introspection_c__StringToString_Request_message_type_support_handle.typesupport_identifier) {
    interface_pkg__srv__StringToString_Request__rosidl_typesupport_introspection_c__StringToString_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &interface_pkg__srv__StringToString_Request__rosidl_typesupport_introspection_c__StringToString_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "interface_pkg/srv/detail/string_to_string__rosidl_typesupport_introspection_c.h"
// already included above
// #include "interface_pkg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "interface_pkg/srv/detail/string_to_string__functions.h"
// already included above
// #include "interface_pkg/srv/detail/string_to_string__struct.h"


// Include directives for member types
// Member `output`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void interface_pkg__srv__StringToString_Response__rosidl_typesupport_introspection_c__StringToString_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interface_pkg__srv__StringToString_Response__init(message_memory);
}

void interface_pkg__srv__StringToString_Response__rosidl_typesupport_introspection_c__StringToString_Response_fini_function(void * message_memory)
{
  interface_pkg__srv__StringToString_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember interface_pkg__srv__StringToString_Response__rosidl_typesupport_introspection_c__StringToString_Response_message_member_array[1] = {
  {
    "output",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_pkg__srv__StringToString_Response, output),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers interface_pkg__srv__StringToString_Response__rosidl_typesupport_introspection_c__StringToString_Response_message_members = {
  "interface_pkg__srv",  // message namespace
  "StringToString_Response",  // message name
  1,  // number of fields
  sizeof(interface_pkg__srv__StringToString_Response),
  false,  // has_any_key_member_
  interface_pkg__srv__StringToString_Response__rosidl_typesupport_introspection_c__StringToString_Response_message_member_array,  // message members
  interface_pkg__srv__StringToString_Response__rosidl_typesupport_introspection_c__StringToString_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  interface_pkg__srv__StringToString_Response__rosidl_typesupport_introspection_c__StringToString_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t interface_pkg__srv__StringToString_Response__rosidl_typesupport_introspection_c__StringToString_Response_message_type_support_handle = {
  0,
  &interface_pkg__srv__StringToString_Response__rosidl_typesupport_introspection_c__StringToString_Response_message_members,
  get_message_typesupport_handle_function,
  &interface_pkg__srv__StringToString_Response__get_type_hash,
  &interface_pkg__srv__StringToString_Response__get_type_description,
  &interface_pkg__srv__StringToString_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface_pkg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_pkg, srv, StringToString_Response)() {
  if (!interface_pkg__srv__StringToString_Response__rosidl_typesupport_introspection_c__StringToString_Response_message_type_support_handle.typesupport_identifier) {
    interface_pkg__srv__StringToString_Response__rosidl_typesupport_introspection_c__StringToString_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &interface_pkg__srv__StringToString_Response__rosidl_typesupport_introspection_c__StringToString_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "interface_pkg/srv/detail/string_to_string__rosidl_typesupport_introspection_c.h"
// already included above
// #include "interface_pkg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "interface_pkg/srv/detail/string_to_string__functions.h"
// already included above
// #include "interface_pkg/srv/detail/string_to_string__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "interface_pkg/srv/string_to_string.h"
// Member `request`
// Member `response`
// already included above
// #include "interface_pkg/srv/detail/string_to_string__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__StringToString_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interface_pkg__srv__StringToString_Event__init(message_memory);
}

void interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__StringToString_Event_fini_function(void * message_memory)
{
  interface_pkg__srv__StringToString_Event__fini(message_memory);
}

size_t interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__size_function__StringToString_Event__request(
  const void * untyped_member)
{
  const interface_pkg__srv__StringToString_Request__Sequence * member =
    (const interface_pkg__srv__StringToString_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__get_const_function__StringToString_Event__request(
  const void * untyped_member, size_t index)
{
  const interface_pkg__srv__StringToString_Request__Sequence * member =
    (const interface_pkg__srv__StringToString_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__get_function__StringToString_Event__request(
  void * untyped_member, size_t index)
{
  interface_pkg__srv__StringToString_Request__Sequence * member =
    (interface_pkg__srv__StringToString_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__fetch_function__StringToString_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const interface_pkg__srv__StringToString_Request * item =
    ((const interface_pkg__srv__StringToString_Request *)
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__get_const_function__StringToString_Event__request(untyped_member, index));
  interface_pkg__srv__StringToString_Request * value =
    (interface_pkg__srv__StringToString_Request *)(untyped_value);
  *value = *item;
}

void interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__assign_function__StringToString_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  interface_pkg__srv__StringToString_Request * item =
    ((interface_pkg__srv__StringToString_Request *)
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__get_function__StringToString_Event__request(untyped_member, index));
  const interface_pkg__srv__StringToString_Request * value =
    (const interface_pkg__srv__StringToString_Request *)(untyped_value);
  *item = *value;
}

bool interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__resize_function__StringToString_Event__request(
  void * untyped_member, size_t size)
{
  interface_pkg__srv__StringToString_Request__Sequence * member =
    (interface_pkg__srv__StringToString_Request__Sequence *)(untyped_member);
  interface_pkg__srv__StringToString_Request__Sequence__fini(member);
  return interface_pkg__srv__StringToString_Request__Sequence__init(member, size);
}

size_t interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__size_function__StringToString_Event__response(
  const void * untyped_member)
{
  const interface_pkg__srv__StringToString_Response__Sequence * member =
    (const interface_pkg__srv__StringToString_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__get_const_function__StringToString_Event__response(
  const void * untyped_member, size_t index)
{
  const interface_pkg__srv__StringToString_Response__Sequence * member =
    (const interface_pkg__srv__StringToString_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__get_function__StringToString_Event__response(
  void * untyped_member, size_t index)
{
  interface_pkg__srv__StringToString_Response__Sequence * member =
    (interface_pkg__srv__StringToString_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__fetch_function__StringToString_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const interface_pkg__srv__StringToString_Response * item =
    ((const interface_pkg__srv__StringToString_Response *)
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__get_const_function__StringToString_Event__response(untyped_member, index));
  interface_pkg__srv__StringToString_Response * value =
    (interface_pkg__srv__StringToString_Response *)(untyped_value);
  *value = *item;
}

void interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__assign_function__StringToString_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  interface_pkg__srv__StringToString_Response * item =
    ((interface_pkg__srv__StringToString_Response *)
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__get_function__StringToString_Event__response(untyped_member, index));
  const interface_pkg__srv__StringToString_Response * value =
    (const interface_pkg__srv__StringToString_Response *)(untyped_value);
  *item = *value;
}

bool interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__resize_function__StringToString_Event__response(
  void * untyped_member, size_t size)
{
  interface_pkg__srv__StringToString_Response__Sequence * member =
    (interface_pkg__srv__StringToString_Response__Sequence *)(untyped_member);
  interface_pkg__srv__StringToString_Response__Sequence__fini(member);
  return interface_pkg__srv__StringToString_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__StringToString_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_pkg__srv__StringToString_Event, info),  // bytes offset in struct
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
    offsetof(interface_pkg__srv__StringToString_Event, request),  // bytes offset in struct
    NULL,  // default value
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__size_function__StringToString_Event__request,  // size() function pointer
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__get_const_function__StringToString_Event__request,  // get_const(index) function pointer
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__get_function__StringToString_Event__request,  // get(index) function pointer
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__fetch_function__StringToString_Event__request,  // fetch(index, &value) function pointer
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__assign_function__StringToString_Event__request,  // assign(index, value) function pointer
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__resize_function__StringToString_Event__request  // resize(index) function pointer
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
    offsetof(interface_pkg__srv__StringToString_Event, response),  // bytes offset in struct
    NULL,  // default value
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__size_function__StringToString_Event__response,  // size() function pointer
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__get_const_function__StringToString_Event__response,  // get_const(index) function pointer
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__get_function__StringToString_Event__response,  // get(index) function pointer
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__fetch_function__StringToString_Event__response,  // fetch(index, &value) function pointer
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__assign_function__StringToString_Event__response,  // assign(index, value) function pointer
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__resize_function__StringToString_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__StringToString_Event_message_members = {
  "interface_pkg__srv",  // message namespace
  "StringToString_Event",  // message name
  3,  // number of fields
  sizeof(interface_pkg__srv__StringToString_Event),
  false,  // has_any_key_member_
  interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__StringToString_Event_message_member_array,  // message members
  interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__StringToString_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__StringToString_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__StringToString_Event_message_type_support_handle = {
  0,
  &interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__StringToString_Event_message_members,
  get_message_typesupport_handle_function,
  &interface_pkg__srv__StringToString_Event__get_type_hash,
  &interface_pkg__srv__StringToString_Event__get_type_description,
  &interface_pkg__srv__StringToString_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface_pkg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_pkg, srv, StringToString_Event)() {
  interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__StringToString_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__StringToString_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_pkg, srv, StringToString_Request)();
  interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__StringToString_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_pkg, srv, StringToString_Response)();
  if (!interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__StringToString_Event_message_type_support_handle.typesupport_identifier) {
    interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__StringToString_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__StringToString_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "interface_pkg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "interface_pkg/srv/detail/string_to_string__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers interface_pkg__srv__detail__string_to_string__rosidl_typesupport_introspection_c__StringToString_service_members = {
  "interface_pkg__srv",  // service namespace
  "StringToString",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // interface_pkg__srv__detail__string_to_string__rosidl_typesupport_introspection_c__StringToString_Request_message_type_support_handle,
  NULL,  // response message
  // interface_pkg__srv__detail__string_to_string__rosidl_typesupport_introspection_c__StringToString_Response_message_type_support_handle
  NULL  // event_message
  // interface_pkg__srv__detail__string_to_string__rosidl_typesupport_introspection_c__StringToString_Response_message_type_support_handle
};


static rosidl_service_type_support_t interface_pkg__srv__detail__string_to_string__rosidl_typesupport_introspection_c__StringToString_service_type_support_handle = {
  0,
  &interface_pkg__srv__detail__string_to_string__rosidl_typesupport_introspection_c__StringToString_service_members,
  get_service_typesupport_handle_function,
  &interface_pkg__srv__StringToString_Request__rosidl_typesupport_introspection_c__StringToString_Request_message_type_support_handle,
  &interface_pkg__srv__StringToString_Response__rosidl_typesupport_introspection_c__StringToString_Response_message_type_support_handle,
  &interface_pkg__srv__StringToString_Event__rosidl_typesupport_introspection_c__StringToString_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    interface_pkg,
    srv,
    StringToString
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    interface_pkg,
    srv,
    StringToString
  ),
  &interface_pkg__srv__StringToString__get_type_hash,
  &interface_pkg__srv__StringToString__get_type_description,
  &interface_pkg__srv__StringToString__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_pkg, srv, StringToString_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_pkg, srv, StringToString_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_pkg, srv, StringToString_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface_pkg
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_pkg, srv, StringToString)(void) {
  if (!interface_pkg__srv__detail__string_to_string__rosidl_typesupport_introspection_c__StringToString_service_type_support_handle.typesupport_identifier) {
    interface_pkg__srv__detail__string_to_string__rosidl_typesupport_introspection_c__StringToString_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)interface_pkg__srv__detail__string_to_string__rosidl_typesupport_introspection_c__StringToString_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_pkg, srv, StringToString_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_pkg, srv, StringToString_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_pkg, srv, StringToString_Event)()->data;
  }

  return &interface_pkg__srv__detail__string_to_string__rosidl_typesupport_introspection_c__StringToString_service_type_support_handle;
}
