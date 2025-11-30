// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolo_depth_interfaces:srv/SetTarget.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "yolo_depth_interfaces/srv/set_target.hpp"


#ifndef YOLO_DEPTH_INTERFACES__SRV__DETAIL__SET_TARGET__TRAITS_HPP_
#define YOLO_DEPTH_INTERFACES__SRV__DETAIL__SET_TARGET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yolo_depth_interfaces/srv/detail/set_target__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace yolo_depth_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetTarget_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: target
  {
    out << "target: ";
    rosidl_generator_traits::value_to_yaml(msg.target, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetTarget_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target: ";
    rosidl_generator_traits::value_to_yaml(msg.target, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetTarget_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace yolo_depth_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use yolo_depth_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yolo_depth_interfaces::srv::SetTarget_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  yolo_depth_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yolo_depth_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const yolo_depth_interfaces::srv::SetTarget_Request & msg)
{
  return yolo_depth_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<yolo_depth_interfaces::srv::SetTarget_Request>()
{
  return "yolo_depth_interfaces::srv::SetTarget_Request";
}

template<>
inline const char * name<yolo_depth_interfaces::srv::SetTarget_Request>()
{
  return "yolo_depth_interfaces/srv/SetTarget_Request";
}

template<>
struct has_fixed_size<yolo_depth_interfaces::srv::SetTarget_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<yolo_depth_interfaces::srv::SetTarget_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<yolo_depth_interfaces::srv::SetTarget_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace yolo_depth_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetTarget_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetTarget_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetTarget_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace yolo_depth_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use yolo_depth_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yolo_depth_interfaces::srv::SetTarget_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  yolo_depth_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yolo_depth_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const yolo_depth_interfaces::srv::SetTarget_Response & msg)
{
  return yolo_depth_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<yolo_depth_interfaces::srv::SetTarget_Response>()
{
  return "yolo_depth_interfaces::srv::SetTarget_Response";
}

template<>
inline const char * name<yolo_depth_interfaces::srv::SetTarget_Response>()
{
  return "yolo_depth_interfaces/srv/SetTarget_Response";
}

template<>
struct has_fixed_size<yolo_depth_interfaces::srv::SetTarget_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<yolo_depth_interfaces::srv::SetTarget_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<yolo_depth_interfaces::srv::SetTarget_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace yolo_depth_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetTarget_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
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
  const SetTarget_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetTarget_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace yolo_depth_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use yolo_depth_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yolo_depth_interfaces::srv::SetTarget_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  yolo_depth_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yolo_depth_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const yolo_depth_interfaces::srv::SetTarget_Event & msg)
{
  return yolo_depth_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<yolo_depth_interfaces::srv::SetTarget_Event>()
{
  return "yolo_depth_interfaces::srv::SetTarget_Event";
}

template<>
inline const char * name<yolo_depth_interfaces::srv::SetTarget_Event>()
{
  return "yolo_depth_interfaces/srv/SetTarget_Event";
}

template<>
struct has_fixed_size<yolo_depth_interfaces::srv::SetTarget_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<yolo_depth_interfaces::srv::SetTarget_Event>
  : std::integral_constant<bool, has_bounded_size<service_msgs::msg::ServiceEventInfo>::value && has_bounded_size<yolo_depth_interfaces::srv::SetTarget_Request>::value && has_bounded_size<yolo_depth_interfaces::srv::SetTarget_Response>::value> {};

template<>
struct is_message<yolo_depth_interfaces::srv::SetTarget_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<yolo_depth_interfaces::srv::SetTarget>()
{
  return "yolo_depth_interfaces::srv::SetTarget";
}

template<>
inline const char * name<yolo_depth_interfaces::srv::SetTarget>()
{
  return "yolo_depth_interfaces/srv/SetTarget";
}

template<>
struct has_fixed_size<yolo_depth_interfaces::srv::SetTarget>
  : std::integral_constant<
    bool,
    has_fixed_size<yolo_depth_interfaces::srv::SetTarget_Request>::value &&
    has_fixed_size<yolo_depth_interfaces::srv::SetTarget_Response>::value
  >
{
};

template<>
struct has_bounded_size<yolo_depth_interfaces::srv::SetTarget>
  : std::integral_constant<
    bool,
    has_bounded_size<yolo_depth_interfaces::srv::SetTarget_Request>::value &&
    has_bounded_size<yolo_depth_interfaces::srv::SetTarget_Response>::value
  >
{
};

template<>
struct is_service<yolo_depth_interfaces::srv::SetTarget>
  : std::true_type
{
};

template<>
struct is_service_request<yolo_depth_interfaces::srv::SetTarget_Request>
  : std::true_type
{
};

template<>
struct is_service_response<yolo_depth_interfaces::srv::SetTarget_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // YOLO_DEPTH_INTERFACES__SRV__DETAIL__SET_TARGET__TRAITS_HPP_
