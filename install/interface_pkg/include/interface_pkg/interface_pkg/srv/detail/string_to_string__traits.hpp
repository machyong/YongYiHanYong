// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface_pkg:srv/StringToString.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interface_pkg/srv/string_to_string.hpp"


#ifndef INTERFACE_PKG__SRV__DETAIL__STRING_TO_STRING__TRAITS_HPP_
#define INTERFACE_PKG__SRV__DETAIL__STRING_TO_STRING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interface_pkg/srv/detail/string_to_string__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interface_pkg
{

namespace srv
{

inline void to_flow_style_yaml(
  const StringToString_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: input
  {
    out << "input: ";
    rosidl_generator_traits::value_to_yaml(msg.input, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StringToString_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: input
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "input: ";
    rosidl_generator_traits::value_to_yaml(msg.input, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StringToString_Request & msg, bool use_flow_style = false)
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

}  // namespace interface_pkg

namespace rosidl_generator_traits
{

[[deprecated("use interface_pkg::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interface_pkg::srv::StringToString_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  interface_pkg::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interface_pkg::srv::to_yaml() instead")]]
inline std::string to_yaml(const interface_pkg::srv::StringToString_Request & msg)
{
  return interface_pkg::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interface_pkg::srv::StringToString_Request>()
{
  return "interface_pkg::srv::StringToString_Request";
}

template<>
inline const char * name<interface_pkg::srv::StringToString_Request>()
{
  return "interface_pkg/srv/StringToString_Request";
}

template<>
struct has_fixed_size<interface_pkg::srv::StringToString_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interface_pkg::srv::StringToString_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interface_pkg::srv::StringToString_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace interface_pkg
{

namespace srv
{

inline void to_flow_style_yaml(
  const StringToString_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: output
  {
    out << "output: ";
    rosidl_generator_traits::value_to_yaml(msg.output, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StringToString_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: output
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output: ";
    rosidl_generator_traits::value_to_yaml(msg.output, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StringToString_Response & msg, bool use_flow_style = false)
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

}  // namespace interface_pkg

namespace rosidl_generator_traits
{

[[deprecated("use interface_pkg::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interface_pkg::srv::StringToString_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  interface_pkg::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interface_pkg::srv::to_yaml() instead")]]
inline std::string to_yaml(const interface_pkg::srv::StringToString_Response & msg)
{
  return interface_pkg::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interface_pkg::srv::StringToString_Response>()
{
  return "interface_pkg::srv::StringToString_Response";
}

template<>
inline const char * name<interface_pkg::srv::StringToString_Response>()
{
  return "interface_pkg/srv/StringToString_Response";
}

template<>
struct has_fixed_size<interface_pkg::srv::StringToString_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interface_pkg::srv::StringToString_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interface_pkg::srv::StringToString_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace interface_pkg
{

namespace srv
{

inline void to_flow_style_yaml(
  const StringToString_Event & msg,
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
  const StringToString_Event & msg,
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

inline std::string to_yaml(const StringToString_Event & msg, bool use_flow_style = false)
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

}  // namespace interface_pkg

namespace rosidl_generator_traits
{

[[deprecated("use interface_pkg::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interface_pkg::srv::StringToString_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  interface_pkg::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interface_pkg::srv::to_yaml() instead")]]
inline std::string to_yaml(const interface_pkg::srv::StringToString_Event & msg)
{
  return interface_pkg::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interface_pkg::srv::StringToString_Event>()
{
  return "interface_pkg::srv::StringToString_Event";
}

template<>
inline const char * name<interface_pkg::srv::StringToString_Event>()
{
  return "interface_pkg/srv/StringToString_Event";
}

template<>
struct has_fixed_size<interface_pkg::srv::StringToString_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interface_pkg::srv::StringToString_Event>
  : std::integral_constant<bool, has_bounded_size<interface_pkg::srv::StringToString_Request>::value && has_bounded_size<interface_pkg::srv::StringToString_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<interface_pkg::srv::StringToString_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface_pkg::srv::StringToString>()
{
  return "interface_pkg::srv::StringToString";
}

template<>
inline const char * name<interface_pkg::srv::StringToString>()
{
  return "interface_pkg/srv/StringToString";
}

template<>
struct has_fixed_size<interface_pkg::srv::StringToString>
  : std::integral_constant<
    bool,
    has_fixed_size<interface_pkg::srv::StringToString_Request>::value &&
    has_fixed_size<interface_pkg::srv::StringToString_Response>::value
  >
{
};

template<>
struct has_bounded_size<interface_pkg::srv::StringToString>
  : std::integral_constant<
    bool,
    has_bounded_size<interface_pkg::srv::StringToString_Request>::value &&
    has_bounded_size<interface_pkg::srv::StringToString_Response>::value
  >
{
};

template<>
struct is_service<interface_pkg::srv::StringToString>
  : std::true_type
{
};

template<>
struct is_service_request<interface_pkg::srv::StringToString_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interface_pkg::srv::StringToString_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE_PKG__SRV__DETAIL__STRING_TO_STRING__TRAITS_HPP_
