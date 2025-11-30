// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolo_depth_interfaces:msg/ObjectDepth.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "yolo_depth_interfaces/msg/object_depth.hpp"


#ifndef YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH__BUILDER_HPP_
#define YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yolo_depth_interfaces/msg/detail/object_depth__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yolo_depth_interfaces
{

namespace msg
{

namespace builder
{

class Init_ObjectDepth_y2
{
public:
  explicit Init_ObjectDepth_y2(::yolo_depth_interfaces::msg::ObjectDepth & msg)
  : msg_(msg)
  {}
  ::yolo_depth_interfaces::msg::ObjectDepth y2(::yolo_depth_interfaces::msg::ObjectDepth::_y2_type arg)
  {
    msg_.y2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_depth_interfaces::msg::ObjectDepth msg_;
};

class Init_ObjectDepth_x2
{
public:
  explicit Init_ObjectDepth_x2(::yolo_depth_interfaces::msg::ObjectDepth & msg)
  : msg_(msg)
  {}
  Init_ObjectDepth_y2 x2(::yolo_depth_interfaces::msg::ObjectDepth::_x2_type arg)
  {
    msg_.x2 = std::move(arg);
    return Init_ObjectDepth_y2(msg_);
  }

private:
  ::yolo_depth_interfaces::msg::ObjectDepth msg_;
};

class Init_ObjectDepth_y1
{
public:
  explicit Init_ObjectDepth_y1(::yolo_depth_interfaces::msg::ObjectDepth & msg)
  : msg_(msg)
  {}
  Init_ObjectDepth_x2 y1(::yolo_depth_interfaces::msg::ObjectDepth::_y1_type arg)
  {
    msg_.y1 = std::move(arg);
    return Init_ObjectDepth_x2(msg_);
  }

private:
  ::yolo_depth_interfaces::msg::ObjectDepth msg_;
};

class Init_ObjectDepth_x1
{
public:
  explicit Init_ObjectDepth_x1(::yolo_depth_interfaces::msg::ObjectDepth & msg)
  : msg_(msg)
  {}
  Init_ObjectDepth_y1 x1(::yolo_depth_interfaces::msg::ObjectDepth::_x1_type arg)
  {
    msg_.x1 = std::move(arg);
    return Init_ObjectDepth_y1(msg_);
  }

private:
  ::yolo_depth_interfaces::msg::ObjectDepth msg_;
};

class Init_ObjectDepth_distance_m
{
public:
  explicit Init_ObjectDepth_distance_m(::yolo_depth_interfaces::msg::ObjectDepth & msg)
  : msg_(msg)
  {}
  Init_ObjectDepth_x1 distance_m(::yolo_depth_interfaces::msg::ObjectDepth::_distance_m_type arg)
  {
    msg_.distance_m = std::move(arg);
    return Init_ObjectDepth_x1(msg_);
  }

private:
  ::yolo_depth_interfaces::msg::ObjectDepth msg_;
};

class Init_ObjectDepth_confidence
{
public:
  explicit Init_ObjectDepth_confidence(::yolo_depth_interfaces::msg::ObjectDepth & msg)
  : msg_(msg)
  {}
  Init_ObjectDepth_distance_m confidence(::yolo_depth_interfaces::msg::ObjectDepth::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_ObjectDepth_distance_m(msg_);
  }

private:
  ::yolo_depth_interfaces::msg::ObjectDepth msg_;
};

class Init_ObjectDepth_class_name
{
public:
  Init_ObjectDepth_class_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectDepth_confidence class_name(::yolo_depth_interfaces::msg::ObjectDepth::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_ObjectDepth_confidence(msg_);
  }

private:
  ::yolo_depth_interfaces::msg::ObjectDepth msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_depth_interfaces::msg::ObjectDepth>()
{
  return yolo_depth_interfaces::msg::builder::Init_ObjectDepth_class_name();
}

}  // namespace yolo_depth_interfaces

#endif  // YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH__BUILDER_HPP_
