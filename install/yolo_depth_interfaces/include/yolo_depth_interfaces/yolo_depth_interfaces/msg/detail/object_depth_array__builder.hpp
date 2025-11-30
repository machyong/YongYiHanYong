// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolo_depth_interfaces:msg/ObjectDepthArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "yolo_depth_interfaces/msg/object_depth_array.hpp"


#ifndef YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH_ARRAY__BUILDER_HPP_
#define YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yolo_depth_interfaces/msg/detail/object_depth_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yolo_depth_interfaces
{

namespace msg
{

namespace builder
{

class Init_ObjectDepthArray_objects
{
public:
  Init_ObjectDepthArray_objects()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::yolo_depth_interfaces::msg::ObjectDepthArray objects(::yolo_depth_interfaces::msg::ObjectDepthArray::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_depth_interfaces::msg::ObjectDepthArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_depth_interfaces::msg::ObjectDepthArray>()
{
  return yolo_depth_interfaces::msg::builder::Init_ObjectDepthArray_objects();
}

}  // namespace yolo_depth_interfaces

#endif  // YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH_ARRAY__BUILDER_HPP_
