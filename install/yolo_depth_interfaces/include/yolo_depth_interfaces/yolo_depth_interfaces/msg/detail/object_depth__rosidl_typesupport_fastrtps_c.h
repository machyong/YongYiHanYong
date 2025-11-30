// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from yolo_depth_interfaces:msg/ObjectDepth.idl
// generated code does not contain a copyright notice
#ifndef YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "yolo_depth_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "yolo_depth_interfaces/msg/detail/object_depth__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_yolo_depth_interfaces
bool cdr_serialize_yolo_depth_interfaces__msg__ObjectDepth(
  const yolo_depth_interfaces__msg__ObjectDepth * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_yolo_depth_interfaces
bool cdr_deserialize_yolo_depth_interfaces__msg__ObjectDepth(
  eprosima::fastcdr::Cdr &,
  yolo_depth_interfaces__msg__ObjectDepth * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_yolo_depth_interfaces
size_t get_serialized_size_yolo_depth_interfaces__msg__ObjectDepth(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_yolo_depth_interfaces
size_t max_serialized_size_yolo_depth_interfaces__msg__ObjectDepth(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_yolo_depth_interfaces
bool cdr_serialize_key_yolo_depth_interfaces__msg__ObjectDepth(
  const yolo_depth_interfaces__msg__ObjectDepth * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_yolo_depth_interfaces
size_t get_serialized_size_key_yolo_depth_interfaces__msg__ObjectDepth(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_yolo_depth_interfaces
size_t max_serialized_size_key_yolo_depth_interfaces__msg__ObjectDepth(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_yolo_depth_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yolo_depth_interfaces, msg, ObjectDepth)();

#ifdef __cplusplus
}
#endif

#endif  // YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
