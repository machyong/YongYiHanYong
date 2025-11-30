// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_pkg:srv/GetTargetPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interface_pkg/srv/get_target_point.hpp"


#ifndef INTERFACE_PKG__SRV__DETAIL__GET_TARGET_POINT__BUILDER_HPP_
#define INTERFACE_PKG__SRV__DETAIL__GET_TARGET_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_pkg/srv/detail/get_target_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_pkg
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_pkg::srv::GetTargetPoint_Request>()
{
  return ::interface_pkg::srv::GetTargetPoint_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace interface_pkg


namespace interface_pkg
{

namespace srv
{

namespace builder
{

class Init_GetTargetPoint_Response_confidence
{
public:
  explicit Init_GetTargetPoint_Response_confidence(::interface_pkg::srv::GetTargetPoint_Response & msg)
  : msg_(msg)
  {}
  ::interface_pkg::srv::GetTargetPoint_Response confidence(::interface_pkg::srv::GetTargetPoint_Response::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_pkg::srv::GetTargetPoint_Response msg_;
};

class Init_GetTargetPoint_Response_class_name
{
public:
  explicit Init_GetTargetPoint_Response_class_name(::interface_pkg::srv::GetTargetPoint_Response & msg)
  : msg_(msg)
  {}
  Init_GetTargetPoint_Response_confidence class_name(::interface_pkg::srv::GetTargetPoint_Response::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_GetTargetPoint_Response_confidence(msg_);
  }

private:
  ::interface_pkg::srv::GetTargetPoint_Response msg_;
};

class Init_GetTargetPoint_Response_target_point
{
public:
  Init_GetTargetPoint_Response_target_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTargetPoint_Response_class_name target_point(::interface_pkg::srv::GetTargetPoint_Response::_target_point_type arg)
  {
    msg_.target_point = std::move(arg);
    return Init_GetTargetPoint_Response_class_name(msg_);
  }

private:
  ::interface_pkg::srv::GetTargetPoint_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_pkg::srv::GetTargetPoint_Response>()
{
  return interface_pkg::srv::builder::Init_GetTargetPoint_Response_target_point();
}

}  // namespace interface_pkg


namespace interface_pkg
{

namespace srv
{

namespace builder
{

class Init_GetTargetPoint_Event_response
{
public:
  explicit Init_GetTargetPoint_Event_response(::interface_pkg::srv::GetTargetPoint_Event & msg)
  : msg_(msg)
  {}
  ::interface_pkg::srv::GetTargetPoint_Event response(::interface_pkg::srv::GetTargetPoint_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_pkg::srv::GetTargetPoint_Event msg_;
};

class Init_GetTargetPoint_Event_request
{
public:
  explicit Init_GetTargetPoint_Event_request(::interface_pkg::srv::GetTargetPoint_Event & msg)
  : msg_(msg)
  {}
  Init_GetTargetPoint_Event_response request(::interface_pkg::srv::GetTargetPoint_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetTargetPoint_Event_response(msg_);
  }

private:
  ::interface_pkg::srv::GetTargetPoint_Event msg_;
};

class Init_GetTargetPoint_Event_info
{
public:
  Init_GetTargetPoint_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTargetPoint_Event_request info(::interface_pkg::srv::GetTargetPoint_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetTargetPoint_Event_request(msg_);
  }

private:
  ::interface_pkg::srv::GetTargetPoint_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_pkg::srv::GetTargetPoint_Event>()
{
  return interface_pkg::srv::builder::Init_GetTargetPoint_Event_info();
}

}  // namespace interface_pkg

#endif  // INTERFACE_PKG__SRV__DETAIL__GET_TARGET_POINT__BUILDER_HPP_
