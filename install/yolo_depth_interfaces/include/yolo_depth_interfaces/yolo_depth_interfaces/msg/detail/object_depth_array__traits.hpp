// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolo_depth_interfaces:msg/ObjectDepthArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "yolo_depth_interfaces/msg/object_depth_array.hpp"


#ifndef YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH_ARRAY__TRAITS_HPP_
#define YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yolo_depth_interfaces/msg/detail/object_depth_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'objects'
#include "yolo_depth_interfaces/msg/detail/object_depth__traits.hpp"

namespace yolo_depth_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObjectDepthArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: objects
  {
    if (msg.objects.size() == 0) {
      out << "objects: []";
    } else {
      out << "objects: [";
      size_t pending_items = msg.objects.size();
      for (auto item : msg.objects) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObjectDepthArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: objects
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.objects.size() == 0) {
      out << "objects: []\n";
    } else {
      out << "objects:\n";
      for (auto item : msg.objects) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObjectDepthArray & msg, bool use_flow_style = false)
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
  const yolo_depth_interfaces::msg::ObjectDepthArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  yolo_depth_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yolo_depth_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const yolo_depth_interfaces::msg::ObjectDepthArray & msg)
{
  return yolo_depth_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yolo_depth_interfaces::msg::ObjectDepthArray>()
{
  return "yolo_depth_interfaces::msg::ObjectDepthArray";
}

template<>
inline const char * name<yolo_depth_interfaces::msg::ObjectDepthArray>()
{
  return "yolo_depth_interfaces/msg/ObjectDepthArray";
}

template<>
struct has_fixed_size<yolo_depth_interfaces::msg::ObjectDepthArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<yolo_depth_interfaces::msg::ObjectDepthArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<yolo_depth_interfaces::msg::ObjectDepthArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH_ARRAY__TRAITS_HPP_
