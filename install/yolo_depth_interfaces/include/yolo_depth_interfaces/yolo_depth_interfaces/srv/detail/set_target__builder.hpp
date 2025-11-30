// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolo_depth_interfaces:srv/SetTarget.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "yolo_depth_interfaces/srv/set_target.hpp"


#ifndef YOLO_DEPTH_INTERFACES__SRV__DETAIL__SET_TARGET__BUILDER_HPP_
#define YOLO_DEPTH_INTERFACES__SRV__DETAIL__SET_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yolo_depth_interfaces/srv/detail/set_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yolo_depth_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetTarget_Request_target
{
public:
  Init_SetTarget_Request_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::yolo_depth_interfaces::srv::SetTarget_Request target(::yolo_depth_interfaces::srv::SetTarget_Request::_target_type arg)
  {
    msg_.target = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::SetTarget_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_depth_interfaces::srv::SetTarget_Request>()
{
  return yolo_depth_interfaces::srv::builder::Init_SetTarget_Request_target();
}

}  // namespace yolo_depth_interfaces


namespace yolo_depth_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetTarget_Response_message
{
public:
  explicit Init_SetTarget_Response_message(::yolo_depth_interfaces::srv::SetTarget_Response & msg)
  : msg_(msg)
  {}
  ::yolo_depth_interfaces::srv::SetTarget_Response message(::yolo_depth_interfaces::srv::SetTarget_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::SetTarget_Response msg_;
};

class Init_SetTarget_Response_success
{
public:
  Init_SetTarget_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetTarget_Response_message success(::yolo_depth_interfaces::srv::SetTarget_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetTarget_Response_message(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::SetTarget_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_depth_interfaces::srv::SetTarget_Response>()
{
  return yolo_depth_interfaces::srv::builder::Init_SetTarget_Response_success();
}

}  // namespace yolo_depth_interfaces


namespace yolo_depth_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetTarget_Event_response
{
public:
  explicit Init_SetTarget_Event_response(::yolo_depth_interfaces::srv::SetTarget_Event & msg)
  : msg_(msg)
  {}
  ::yolo_depth_interfaces::srv::SetTarget_Event response(::yolo_depth_interfaces::srv::SetTarget_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::SetTarget_Event msg_;
};

class Init_SetTarget_Event_request
{
public:
  explicit Init_SetTarget_Event_request(::yolo_depth_interfaces::srv::SetTarget_Event & msg)
  : msg_(msg)
  {}
  Init_SetTarget_Event_response request(::yolo_depth_interfaces::srv::SetTarget_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetTarget_Event_response(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::SetTarget_Event msg_;
};

class Init_SetTarget_Event_info
{
public:
  Init_SetTarget_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetTarget_Event_request info(::yolo_depth_interfaces::srv::SetTarget_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetTarget_Event_request(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::SetTarget_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_depth_interfaces::srv::SetTarget_Event>()
{
  return yolo_depth_interfaces::srv::builder::Init_SetTarget_Event_info();
}

}  // namespace yolo_depth_interfaces

#endif  // YOLO_DEPTH_INTERFACES__SRV__DETAIL__SET_TARGET__BUILDER_HPP_
