// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolo_depth_interfaces:srv/MoveToPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "yolo_depth_interfaces/srv/move_to_point.hpp"


#ifndef YOLO_DEPTH_INTERFACES__SRV__DETAIL__MOVE_TO_POINT__BUILDER_HPP_
#define YOLO_DEPTH_INTERFACES__SRV__DETAIL__MOVE_TO_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yolo_depth_interfaces/srv/detail/move_to_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yolo_depth_interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveToPoint_Request_target_position
{
public:
  Init_MoveToPoint_Request_target_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::yolo_depth_interfaces::srv::MoveToPoint_Request target_position(::yolo_depth_interfaces::srv::MoveToPoint_Request::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::MoveToPoint_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_depth_interfaces::srv::MoveToPoint_Request>()
{
  return yolo_depth_interfaces::srv::builder::Init_MoveToPoint_Request_target_position();
}

}  // namespace yolo_depth_interfaces


namespace yolo_depth_interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveToPoint_Response_message
{
public:
  explicit Init_MoveToPoint_Response_message(::yolo_depth_interfaces::srv::MoveToPoint_Response & msg)
  : msg_(msg)
  {}
  ::yolo_depth_interfaces::srv::MoveToPoint_Response message(::yolo_depth_interfaces::srv::MoveToPoint_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::MoveToPoint_Response msg_;
};

class Init_MoveToPoint_Response_success
{
public:
  Init_MoveToPoint_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToPoint_Response_message success(::yolo_depth_interfaces::srv::MoveToPoint_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MoveToPoint_Response_message(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::MoveToPoint_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_depth_interfaces::srv::MoveToPoint_Response>()
{
  return yolo_depth_interfaces::srv::builder::Init_MoveToPoint_Response_success();
}

}  // namespace yolo_depth_interfaces


namespace yolo_depth_interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveToPoint_Event_response
{
public:
  explicit Init_MoveToPoint_Event_response(::yolo_depth_interfaces::srv::MoveToPoint_Event & msg)
  : msg_(msg)
  {}
  ::yolo_depth_interfaces::srv::MoveToPoint_Event response(::yolo_depth_interfaces::srv::MoveToPoint_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::MoveToPoint_Event msg_;
};

class Init_MoveToPoint_Event_request
{
public:
  explicit Init_MoveToPoint_Event_request(::yolo_depth_interfaces::srv::MoveToPoint_Event & msg)
  : msg_(msg)
  {}
  Init_MoveToPoint_Event_response request(::yolo_depth_interfaces::srv::MoveToPoint_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_MoveToPoint_Event_response(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::MoveToPoint_Event msg_;
};

class Init_MoveToPoint_Event_info
{
public:
  Init_MoveToPoint_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToPoint_Event_request info(::yolo_depth_interfaces::srv::MoveToPoint_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_MoveToPoint_Event_request(msg_);
  }

private:
  ::yolo_depth_interfaces::srv::MoveToPoint_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_depth_interfaces::srv::MoveToPoint_Event>()
{
  return yolo_depth_interfaces::srv::builder::Init_MoveToPoint_Event_info();
}

}  // namespace yolo_depth_interfaces

#endif  // YOLO_DEPTH_INTERFACES__SRV__DETAIL__MOVE_TO_POINT__BUILDER_HPP_
