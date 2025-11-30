// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yolo_depth_interfaces:srv/TargetCenter.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "yolo_depth_interfaces/srv/target_center.hpp"


#ifndef YOLO_DEPTH_INTERFACES__SRV__DETAIL__TARGET_CENTER__STRUCT_HPP_
#define YOLO_DEPTH_INTERFACES__SRV__DETAIL__TARGET_CENTER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__yolo_depth_interfaces__srv__TargetCenter_Request __attribute__((deprecated))
#else
# define DEPRECATED__yolo_depth_interfaces__srv__TargetCenter_Request __declspec(deprecated)
#endif

namespace yolo_depth_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TargetCenter_Request_
{
  using Type = TargetCenter_Request_<ContainerAllocator>;

  explicit TargetCenter_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target = "";
      this->center_x = 0.0;
      this->center_y = 0.0;
      this->distance_m = 0.0;
    }
  }

  explicit TargetCenter_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target = "";
      this->center_x = 0.0;
      this->center_y = 0.0;
      this->distance_m = 0.0;
    }
  }

  // field types and members
  using _target_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _target_type target;
  using _center_x_type =
    double;
  _center_x_type center_x;
  using _center_y_type =
    double;
  _center_y_type center_y;
  using _distance_m_type =
    double;
  _distance_m_type distance_m;

  // setters for named parameter idiom
  Type & set__target(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->target = _arg;
    return *this;
  }
  Type & set__center_x(
    const double & _arg)
  {
    this->center_x = _arg;
    return *this;
  }
  Type & set__center_y(
    const double & _arg)
  {
    this->center_y = _arg;
    return *this;
  }
  Type & set__distance_m(
    const double & _arg)
  {
    this->distance_m = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yolo_depth_interfaces::srv::TargetCenter_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const yolo_depth_interfaces::srv::TargetCenter_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yolo_depth_interfaces::srv::TargetCenter_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yolo_depth_interfaces::srv::TargetCenter_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yolo_depth_interfaces::srv::TargetCenter_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yolo_depth_interfaces::srv::TargetCenter_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yolo_depth_interfaces::srv::TargetCenter_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yolo_depth_interfaces::srv::TargetCenter_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yolo_depth_interfaces::srv::TargetCenter_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yolo_depth_interfaces::srv::TargetCenter_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yolo_depth_interfaces__srv__TargetCenter_Request
    std::shared_ptr<yolo_depth_interfaces::srv::TargetCenter_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yolo_depth_interfaces__srv__TargetCenter_Request
    std::shared_ptr<yolo_depth_interfaces::srv::TargetCenter_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TargetCenter_Request_ & other) const
  {
    if (this->target != other.target) {
      return false;
    }
    if (this->center_x != other.center_x) {
      return false;
    }
    if (this->center_y != other.center_y) {
      return false;
    }
    if (this->distance_m != other.distance_m) {
      return false;
    }
    return true;
  }
  bool operator!=(const TargetCenter_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TargetCenter_Request_

// alias to use template instance with default allocator
using TargetCenter_Request =
  yolo_depth_interfaces::srv::TargetCenter_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace yolo_depth_interfaces


#ifndef _WIN32
# define DEPRECATED__yolo_depth_interfaces__srv__TargetCenter_Response __attribute__((deprecated))
#else
# define DEPRECATED__yolo_depth_interfaces__srv__TargetCenter_Response __declspec(deprecated)
#endif

namespace yolo_depth_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TargetCenter_Response_
{
  using Type = TargetCenter_Response_<ContainerAllocator>;

  explicit TargetCenter_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit TargetCenter_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yolo_depth_interfaces::srv::TargetCenter_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const yolo_depth_interfaces::srv::TargetCenter_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yolo_depth_interfaces::srv::TargetCenter_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yolo_depth_interfaces::srv::TargetCenter_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yolo_depth_interfaces::srv::TargetCenter_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yolo_depth_interfaces::srv::TargetCenter_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yolo_depth_interfaces::srv::TargetCenter_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yolo_depth_interfaces::srv::TargetCenter_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yolo_depth_interfaces::srv::TargetCenter_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yolo_depth_interfaces::srv::TargetCenter_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yolo_depth_interfaces__srv__TargetCenter_Response
    std::shared_ptr<yolo_depth_interfaces::srv::TargetCenter_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yolo_depth_interfaces__srv__TargetCenter_Response
    std::shared_ptr<yolo_depth_interfaces::srv::TargetCenter_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TargetCenter_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const TargetCenter_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TargetCenter_Response_

// alias to use template instance with default allocator
using TargetCenter_Response =
  yolo_depth_interfaces::srv::TargetCenter_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace yolo_depth_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__yolo_depth_interfaces__srv__TargetCenter_Event __attribute__((deprecated))
#else
# define DEPRECATED__yolo_depth_interfaces__srv__TargetCenter_Event __declspec(deprecated)
#endif

namespace yolo_depth_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TargetCenter_Event_
{
  using Type = TargetCenter_Event_<ContainerAllocator>;

  explicit TargetCenter_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit TargetCenter_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<yolo_depth_interfaces::srv::TargetCenter_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<yolo_depth_interfaces::srv::TargetCenter_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<yolo_depth_interfaces::srv::TargetCenter_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<yolo_depth_interfaces::srv::TargetCenter_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<yolo_depth_interfaces::srv::TargetCenter_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<yolo_depth_interfaces::srv::TargetCenter_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<yolo_depth_interfaces::srv::TargetCenter_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<yolo_depth_interfaces::srv::TargetCenter_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yolo_depth_interfaces::srv::TargetCenter_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const yolo_depth_interfaces::srv::TargetCenter_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yolo_depth_interfaces::srv::TargetCenter_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yolo_depth_interfaces::srv::TargetCenter_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yolo_depth_interfaces::srv::TargetCenter_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yolo_depth_interfaces::srv::TargetCenter_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yolo_depth_interfaces::srv::TargetCenter_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yolo_depth_interfaces::srv::TargetCenter_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yolo_depth_interfaces::srv::TargetCenter_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yolo_depth_interfaces::srv::TargetCenter_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yolo_depth_interfaces__srv__TargetCenter_Event
    std::shared_ptr<yolo_depth_interfaces::srv::TargetCenter_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yolo_depth_interfaces__srv__TargetCenter_Event
    std::shared_ptr<yolo_depth_interfaces::srv::TargetCenter_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TargetCenter_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const TargetCenter_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TargetCenter_Event_

// alias to use template instance with default allocator
using TargetCenter_Event =
  yolo_depth_interfaces::srv::TargetCenter_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace yolo_depth_interfaces

namespace yolo_depth_interfaces
{

namespace srv
{

struct TargetCenter
{
  using Request = yolo_depth_interfaces::srv::TargetCenter_Request;
  using Response = yolo_depth_interfaces::srv::TargetCenter_Response;
  using Event = yolo_depth_interfaces::srv::TargetCenter_Event;
};

}  // namespace srv

}  // namespace yolo_depth_interfaces

#endif  // YOLO_DEPTH_INTERFACES__SRV__DETAIL__TARGET_CENTER__STRUCT_HPP_
