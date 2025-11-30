// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolo_depth_interfaces:msg/ObjectDepth.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "yolo_depth_interfaces/msg/object_depth.hpp"


#ifndef YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH__TRAITS_HPP_
#define YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yolo_depth_interfaces/msg/detail/object_depth__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace yolo_depth_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObjectDepth & msg,
  std::ostream & out)
{
  out << "{";
  // member: class_name
  {
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: distance_m
  {
    out << "distance_m: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_m, out);
    out << ", ";
  }

  // member: x1
  {
    out << "x1: ";
    rosidl_generator_traits::value_to_yaml(msg.x1, out);
    out << ", ";
  }

  // member: y1
  {
    out << "y1: ";
    rosidl_generator_traits::value_to_yaml(msg.y1, out);
    out << ", ";
  }

  // member: x2
  {
    out << "x2: ";
    rosidl_generator_traits::value_to_yaml(msg.x2, out);
    out << ", ";
  }

  // member: y2
  {
    out << "y2: ";
    rosidl_generator_traits::value_to_yaml(msg.y2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObjectDepth & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: class_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: distance_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_m: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_m, out);
    out << "\n";
  }

  // member: x1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x1: ";
    rosidl_generator_traits::value_to_yaml(msg.x1, out);
    out << "\n";
  }

  // member: y1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y1: ";
    rosidl_generator_traits::value_to_yaml(msg.y1, out);
    out << "\n";
  }

  // member: x2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x2: ";
    rosidl_generator_traits::value_to_yaml(msg.x2, out);
    out << "\n";
  }

  // member: y2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y2: ";
    rosidl_generator_traits::value_to_yaml(msg.y2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObjectDepth & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace yolo_depth_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use yolo_depth_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yolo_depth_interfaces::msg::ObjectDepth & msg,
  std::ostream & out, size_t indentation = 0)
{
  yolo_depth_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yolo_depth_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const yolo_depth_interfaces::msg::ObjectDepth & msg)
{
  return yolo_depth_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yolo_depth_interfaces::msg::ObjectDepth>()
{
  return "yolo_depth_interfaces::msg::ObjectDepth";
}

template<>
inline const char * name<yolo_depth_interfaces::msg::ObjectDepth>()
{
  return "yolo_depth_interfaces/msg/ObjectDepth";
}

template<>
struct has_fixed_size<yolo_depth_interfaces::msg::ObjectDepth>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<yolo_depth_interfaces::msg::ObjectDepth>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<yolo_depth_interfaces::msg::ObjectDepth>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH__TRAITS_HPP_
