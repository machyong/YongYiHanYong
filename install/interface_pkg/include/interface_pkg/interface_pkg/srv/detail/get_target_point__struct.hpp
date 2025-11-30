// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface_pkg:srv/GetTargetPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interface_pkg/srv/get_target_point.hpp"


#ifndef INTERFACE_PKG__SRV__DETAIL__GET_TARGET_POINT__STRUCT_HPP_
#define INTERFACE_PKG__SRV__DETAIL__GET_TARGET_POINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interface_pkg__srv__GetTargetPoint_Request __attribute__((deprecated))
#else
# define DEPRECATED__interface_pkg__srv__GetTargetPoint_Request __declspec(deprecated)
#endif

namespace interface_pkg
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTargetPoint_Request_
{
  using Type = GetTargetPoint_Request_<ContainerAllocator>;

  explicit GetTargetPoint_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetTargetPoint_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    interface_pkg::srv::GetTargetPoint_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface_pkg::srv::GetTargetPoint_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface_pkg::srv::GetTargetPoint_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface_pkg::srv::GetTargetPoint_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface_pkg::srv::GetTargetPoint_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface_pkg::srv::GetTargetPoint_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface_pkg::srv::GetTargetPoint_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface_pkg::srv::GetTargetPoint_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface_pkg::srv::GetTargetPoint_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface_pkg::srv::GetTargetPoint_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface_pkg__srv__GetTargetPoint_Request
    std::shared_ptr<interface_pkg::srv::GetTargetPoint_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface_pkg__srv__GetTargetPoint_Request
    std::shared_ptr<interface_pkg::srv::GetTargetPoint_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTargetPoint_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetTargetPoint_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTargetPoint_Request_

// alias to use template instance with default allocator
using GetTargetPoint_Request =
  interface_pkg::srv::GetTargetPoint_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface_pkg


// Include directives for member types
// Member 'target_point'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__interface_pkg__srv__GetTargetPoint_Response __attribute__((deprecated))
#else
# define DEPRECATED__interface_pkg__srv__GetTargetPoint_Response __declspec(deprecated)
#endif

namespace interface_pkg
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTargetPoint_Response_
{
  using Type = GetTargetPoint_Response_<ContainerAllocator>;

  explicit GetTargetPoint_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_point(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_name = "";
      this->confidence = 0.0f;
    }
  }

  explicit GetTargetPoint_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_point(_alloc, _init),
    class_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_name = "";
      this->confidence = 0.0f;
    }
  }

  // field types and members
  using _target_point_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _target_point_type target_point;
  using _class_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _class_name_type class_name;
  using _confidence_type =
    float;
  _confidence_type confidence;

  // setters for named parameter idiom
  Type & set__target_point(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->target_point = _arg;
    return *this;
  }
  Type & set__class_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->class_name = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface_pkg::srv::GetTargetPoint_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface_pkg::srv::GetTargetPoint_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface_pkg::srv::GetTargetPoint_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface_pkg::srv::GetTargetPoint_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface_pkg::srv::GetTargetPoint_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface_pkg::srv::GetTargetPoint_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface_pkg::srv::GetTargetPoint_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface_pkg::srv::GetTargetPoint_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface_pkg::srv::GetTargetPoint_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface_pkg::srv::GetTargetPoint_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface_pkg__srv__GetTargetPoint_Response
    std::shared_ptr<interface_pkg::srv::GetTargetPoint_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface_pkg__srv__GetTargetPoint_Response
    std::shared_ptr<interface_pkg::srv::GetTargetPoint_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTargetPoint_Response_ & other) const
  {
    if (this->target_point != other.target_point) {
      return false;
    }
    if (this->class_name != other.class_name) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetTargetPoint_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTargetPoint_Response_

// alias to use template instance with default allocator
using GetTargetPoint_Response =
  interface_pkg::srv::GetTargetPoint_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface_pkg


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__interface_pkg__srv__GetTargetPoint_Event __attribute__((deprecated))
#else
# define DEPRECATED__interface_pkg__srv__GetTargetPoint_Event __declspec(deprecated)
#endif

namespace interface_pkg
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTargetPoint_Event_
{
  using Type = GetTargetPoint_Event_<ContainerAllocator>;

  explicit GetTargetPoint_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetTargetPoint_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<interface_pkg::srv::GetTargetPoint_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<interface_pkg::srv::GetTargetPoint_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<interface_pkg::srv::GetTargetPoint_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<interface_pkg::srv::GetTargetPoint_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<interface_pkg::srv::GetTargetPoint_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<interface_pkg::srv::GetTargetPoint_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<interface_pkg::srv::GetTargetPoint_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<interface_pkg::srv::GetTargetPoint_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface_pkg::srv::GetTargetPoint_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface_pkg::srv::GetTargetPoint_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface_pkg::srv::GetTargetPoint_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface_pkg::srv::GetTargetPoint_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface_pkg::srv::GetTargetPoint_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface_pkg::srv::GetTargetPoint_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface_pkg::srv::GetTargetPoint_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface_pkg::srv::GetTargetPoint_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface_pkg::srv::GetTargetPoint_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface_pkg::srv::GetTargetPoint_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface_pkg__srv__GetTargetPoint_Event
    std::shared_ptr<interface_pkg::srv::GetTargetPoint_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface_pkg__srv__GetTargetPoint_Event
    std::shared_ptr<interface_pkg::srv::GetTargetPoint_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTargetPoint_Event_ & other) const
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
  bool operator!=(const GetTargetPoint_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTargetPoint_Event_

// alias to use template instance with default allocator
using GetTargetPoint_Event =
  interface_pkg::srv::GetTargetPoint_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface_pkg

namespace interface_pkg
{

namespace srv
{

struct GetTargetPoint
{
  using Request = interface_pkg::srv::GetTargetPoint_Request;
  using Response = interface_pkg::srv::GetTargetPoint_Response;
  using Event = interface_pkg::srv::GetTargetPoint_Event;
};

}  // namespace srv

}  // namespace interface_pkg

#endif  // INTERFACE_PKG__SRV__DETAIL__GET_TARGET_POINT__STRUCT_HPP_
