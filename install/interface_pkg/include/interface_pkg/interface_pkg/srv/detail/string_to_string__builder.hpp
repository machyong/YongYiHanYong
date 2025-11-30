// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_pkg:srv/StringToString.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interface_pkg/srv/string_to_string.hpp"


#ifndef INTERFACE_PKG__SRV__DETAIL__STRING_TO_STRING__BUILDER_HPP_
#define INTERFACE_PKG__SRV__DETAIL__STRING_TO_STRING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_pkg/srv/detail/string_to_string__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_pkg
{

namespace srv
{

namespace builder
{

class Init_StringToString_Request_input
{
public:
  Init_StringToString_Request_input()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interface_pkg::srv::StringToString_Request input(::interface_pkg::srv::StringToString_Request::_input_type arg)
  {
    msg_.input = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_pkg::srv::StringToString_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_pkg::srv::StringToString_Request>()
{
  return interface_pkg::srv::builder::Init_StringToString_Request_input();
}

}  // namespace interface_pkg


namespace interface_pkg
{

namespace srv
{

namespace builder
{

class Init_StringToString_Response_output
{
public:
  Init_StringToString_Response_output()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interface_pkg::srv::StringToString_Response output(::interface_pkg::srv::StringToString_Response::_output_type arg)
  {
    msg_.output = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_pkg::srv::StringToString_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_pkg::srv::StringToString_Response>()
{
  return interface_pkg::srv::builder::Init_StringToString_Response_output();
}

}  // namespace interface_pkg


namespace interface_pkg
{

namespace srv
{

namespace builder
{

class Init_StringToString_Event_response
{
public:
  explicit Init_StringToString_Event_response(::interface_pkg::srv::StringToString_Event & msg)
  : msg_(msg)
  {}
  ::interface_pkg::srv::StringToString_Event response(::interface_pkg::srv::StringToString_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_pkg::srv::StringToString_Event msg_;
};

class Init_StringToString_Event_request
{
public:
  explicit Init_StringToString_Event_request(::interface_pkg::srv::StringToString_Event & msg)
  : msg_(msg)
  {}
  Init_StringToString_Event_response request(::interface_pkg::srv::StringToString_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_StringToString_Event_response(msg_);
  }

private:
  ::interface_pkg::srv::StringToString_Event msg_;
};

class Init_StringToString_Event_info
{
public:
  Init_StringToString_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StringToString_Event_request info(::interface_pkg::srv::StringToString_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_StringToString_Event_request(msg_);
  }

private:
  ::interface_pkg::srv::StringToString_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_pkg::srv::StringToString_Event>()
{
  return interface_pkg::srv::builder::Init_StringToString_Event_info();
}

}  // namespace interface_pkg

#endif  // INTERFACE_PKG__SRV__DETAIL__STRING_TO_STRING__BUILDER_HPP_
