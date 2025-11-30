// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolo_depth_interfaces:srv/TargetCenter.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "yolo_depth_interfaces/srv/target_center.hpp"


#ifndef YOLO_DEPTH_INTERFACES__SRV__DETAIL__TARGET_CENTER__BUILDER_HPP_
#define YOLO_DEPTH_INTERFACES__SRV__DETAIL__TARGET_CENTER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yolo_depth_interfaces/srv/detail/target_center__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yolo_depth_interfaces
{

namespace srv
{

namespace builder
{

class Init_TargetCenter_Request_distance_m
{
public:
  explicit Init_TargetCenter_Request_distance_m(::yolo_depth_interfaces::srv::TargetCenter_Request & msg)
  : msg_(msg)
  {}
  ::yolo_depth_interfaces::srv::TargetCenter_Request distance_m(::yolo_depth_interfaces::srv::TargetCenter_Request::_distance_m_type arg)
  {
    msg_.distance_m = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::TargetCenter_Request msg_;
};

class Init_TargetCenter_Request_center_y
{
public:
  explicit Init_TargetCenter_Request_center_y(::yolo_depth_interfaces::srv::TargetCenter_Request & msg)
  : msg_(msg)
  {}
  Init_TargetCenter_Request_distance_m center_y(::yolo_depth_interfaces::srv::TargetCenter_Request::_center_y_type arg)
  {
    msg_.center_y = std::move(arg);
    return Init_TargetCenter_Request_distance_m(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::TargetCenter_Request msg_;
};

class Init_TargetCenter_Request_center_x
{
public:
  explicit Init_TargetCenter_Request_center_x(::yolo_depth_interfaces::srv::TargetCenter_Request & msg)
  : msg_(msg)
  {}
  Init_TargetCenter_Request_center_y center_x(::yolo_depth_interfaces::srv::TargetCenter_Request::_center_x_type arg)
  {
    msg_.center_x = std::move(arg);
    return Init_TargetCenter_Request_center_y(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::TargetCenter_Request msg_;
};

class Init_TargetCenter_Request_target
{
public:
  Init_TargetCenter_Request_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetCenter_Request_center_x target(::yolo_depth_interfaces::srv::TargetCenter_Request::_target_type arg)
  {
    msg_.target = std::move(arg);
    return Init_TargetCenter_Request_center_x(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::TargetCenter_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_depth_interfaces::srv::TargetCenter_Request>()
{
  return yolo_depth_interfaces::srv::builder::Init_TargetCenter_Request_target();
}

}  // namespace yolo_depth_interfaces


namespace yolo_depth_interfaces
{

namespace srv
{

namespace builder
{

class Init_TargetCenter_Response_message
{
public:
  explicit Init_TargetCenter_Response_message(::yolo_depth_interfaces::srv::TargetCenter_Response & msg)
  : msg_(msg)
  {}
  ::yolo_depth_interfaces::srv::TargetCenter_Response message(::yolo_depth_interfaces::srv::TargetCenter_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::TargetCenter_Response msg_;
};

class Init_TargetCenter_Response_success
{
public:
  Init_TargetCenter_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetCenter_Response_message success(::yolo_depth_interfaces::srv::TargetCenter_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_TargetCenter_Response_message(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::TargetCenter_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_depth_interfaces::srv::TargetCenter_Response>()
{
  return yolo_depth_interfaces::srv::builder::Init_TargetCenter_Response_success();
}

}  // namespace yolo_depth_interfaces


namespace yolo_depth_interfaces
{

namespace srv
{

namespace builder
{

class Init_TargetCenter_Event_response
{
public:
  explicit Init_TargetCenter_Event_response(::yolo_depth_interfaces::srv::TargetCenter_Event & msg)
  : msg_(msg)
  {}
  ::yolo_depth_interfaces::srv::TargetCenter_Event response(::yolo_depth_interfaces::srv::TargetCenter_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::TargetCenter_Event msg_;
};

class Init_TargetCenter_Event_request
{
public:
  explicit Init_TargetCenter_Event_request(::yolo_depth_interfaces::srv::TargetCenter_Event & msg)
  : msg_(msg)
  {}
  Init_TargetCenter_Event_response request(::yolo_depth_interfaces::srv::TargetCenter_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_TargetCenter_Event_response(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::TargetCenter_Event msg_;
};

class Init_TargetCenter_Event_info
{
public:
  Init_TargetCenter_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetCenter_Event_request info(::yolo_depth_interfaces::srv::TargetCenter_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_TargetCenter_Event_request(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::TargetCenter_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_depth_interfaces::srv::TargetCenter_Event>()
{
  return yolo_depth_interfaces::srv::builder::Init_TargetCenter_Event_info();
}

}  // namespace yolo_depth_interfaces

#endif  // YOLO_DEPTH_INTERFACES__SRV__DETAIL__TARGET_CENTER__BUILDER_HPP_
