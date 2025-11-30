// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from yolo_depth_interfaces:msg/ObjectDepthArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "yolo_depth_interfaces/msg/detail/object_depth_array__rosidl_typesupport_introspection_c.h"
#include "yolo_depth_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "yolo_depth_interfaces/msg/detail/object_depth_array__functions.h"
#include "yolo_depth_interfaces/msg/detail/object_depth_array__struct.h"


// Include directives for member types
// Member `objects`
#include "yolo_depth_interfaces/msg/object_depth.h"
// Member `objects`
#include "yolo_depth_interfaces/msg/detail/object_depth__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__ObjectDepthArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yolo_depth_interfaces__msg__ObjectDepthArray__init(message_memory);
}

void yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__ObjectDepthArray_fini_function(void * message_memory)
{
  yolo_depth_interfaces__msg__ObjectDepthArray__fini(message_memory);
}

size_t yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__size_function__ObjectDepthArray__objects(
  const void * untyped_member)
{
  const yolo_depth_interfaces__msg__ObjectDepth__Sequence * member =
    (const yolo_depth_interfaces__msg__ObjectDepth__Sequence *)(untyped_member);
  return member->size;
}

const void * yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__get_const_function__ObjectDepthArray__objects(
  const void * untyped_member, size_t index)
{
  const yolo_depth_interfaces__msg__ObjectDepth__Sequence * member =
    (const yolo_depth_interfaces__msg__ObjectDepth__Sequence *)(untyped_member);
  return &member->data[index];
}

void * yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__get_function__ObjectDepthArray__objects(
  void * untyped_member, size_t index)
{
  yolo_depth_interfaces__msg__ObjectDepth__Sequence * member =
    (yolo_depth_interfaces__msg__ObjectDepth__Sequence *)(untyped_member);
  return &member->data[index];
}

void yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__fetch_function__ObjectDepthArray__objects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const yolo_depth_interfaces__msg__ObjectDepth * item =
    ((const yolo_depth_interfaces__msg__ObjectDepth *)
    yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__get_const_function__ObjectDepthArray__objects(untyped_member, index));
  yolo_depth_interfaces__msg__ObjectDepth * value =
    (yolo_depth_interfaces__msg__ObjectDepth *)(untyped_value);
  *value = *item;
}

void yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__assign_function__ObjectDepthArray__objects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  yolo_depth_interfaces__msg__ObjectDepth * item =
    ((yolo_depth_interfaces__msg__ObjectDepth *)
    yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__get_function__ObjectDepthArray__objects(untyped_member, index));
  const yolo_depth_interfaces__msg__ObjectDepth * value =
    (const yolo_depth_interfaces__msg__ObjectDepth *)(untyped_value);
  *item = *value;
}

bool yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__resize_function__ObjectDepthArray__objects(
  void * untyped_member, size_t size)
{
  yolo_depth_interfaces__msg__ObjectDepth__Sequence * member =
    (yolo_depth_interfaces__msg__ObjectDepth__Sequence *)(untyped_member);
  yolo_depth_interfaces__msg__ObjectDepth__Sequence__fini(member);
  return yolo_depth_interfaces__msg__ObjectDepth__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__ObjectDepthArray_message_member_array[1] = {
  {
    "objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_depth_interfaces__msg__ObjectDepthArray, objects),  // bytes offset in struct
    NULL,  // default value
    yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__size_function__ObjectDepthArray__objects,  // size() function pointer
    yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__get_const_function__ObjectDepthArray__objects,  // get_const(index) function pointer
    yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__get_function__ObjectDepthArray__objects,  // get(index) function pointer
    yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__fetch_function__ObjectDepthArray__objects,  // fetch(index, &value) function pointer
    yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__assign_function__ObjectDepthArray__objects,  // assign(index, value) function pointer
    yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__resize_function__ObjectDepthArray__objects  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__ObjectDepthArray_message_members = {
  "yolo_depth_interfaces__msg",  // message namespace
  "ObjectDepthArray",  // message name
  1,  // number of fields
  sizeof(yolo_depth_interfaces__msg__ObjectDepthArray),
  false,  // has_any_key_member_
  yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__ObjectDepthArray_message_member_array,  // message members
  yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__ObjectDepthArray_init_function,  // function to initialize message memory (memory has to be allocated)
  yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__ObjectDepthArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__ObjectDepthArray_message_type_support_handle = {
  0,
  &yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__ObjectDepthArray_message_members,
  get_message_typesupport_handle_function,
  &yolo_depth_interfaces__msg__ObjectDepthArray__get_type_hash,
  &yolo_depth_interfaces__msg__ObjectDepthArray__get_type_description,
  &yolo_depth_interfaces__msg__ObjectDepthArray__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolo_depth_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_depth_interfaces, msg, ObjectDepthArray)() {
  yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__ObjectDepthArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_depth_interfaces, msg, ObjectDepth)();
  if (!yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__ObjectDepthArray_message_type_support_handle.typesupport_identifier) {
    yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__ObjectDepthArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &yolo_depth_interfaces__msg__ObjectDepthArray__rosidl_typesupport_introspection_c__ObjectDepthArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
