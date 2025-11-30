// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface_pkg:srv/StringToString.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interface_pkg/srv/string_to_string.hpp"


#ifndef INTERFACE_PKG__SRV__DETAIL__STRING_TO_STRING__STRUCT_HPP_
#define INTERFACE_PKG__SRV__DETAIL__STRING_TO_STRING__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interface_pkg__srv__StringToString_Request __attribute__((deprecated))
#else
# define DEPRECATED__interface_pkg__srv__StringToString_Request __declspec(deprecated)
#endif

namespace interface_pkg
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StringToString_Request_
{
  using Type = StringToString_Request_<ContainerAllocator>;

  explicit StringToString_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->input = "";
    }
  }

  explicit StringToString_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : input(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->input = "";
    }
  }

  // field types and members
  using _input_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _input_type input;

  // setters for named parameter idiom
  Type & set__input(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->input = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface_pkg::srv::StringToString_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface_pkg::srv::StringToString_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface_pkg::srv::StringToString_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface_pkg::srv::StringToString_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface_pkg::srv::StringToString_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface_pkg::srv::StringToString_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface_pkg::srv::StringToString_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface_pkg::srv::StringToString_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface_pkg::srv::StringToString_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface_pkg::srv::StringToString_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface_pkg__srv__StringToString_Request
    std::shared_ptr<interface_pkg::srv::StringToString_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface_pkg__srv__StringToString_Request
    std::shared_ptr<interface_pkg::srv::StringToString_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StringToString_Request_ & other) const
  {
    if (this->input != other.input) {
      return false;
    }
    return true;
  }
  bool operator!=(const StringToString_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StringToString_Request_

// alias to use template instance with default allocator
using StringToString_Request =
  interface_pkg::srv::StringToString_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface_pkg


#ifndef _WIN32
# define DEPRECATED__interface_pkg__srv__StringToString_Response __attribute__((deprecated))
#else
# define DEPRECATED__interface_pkg__srv__StringToString_Response __declspec(deprecated)
#endif

namespace interface_pkg
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StringToString_Response_
{
  using Type = StringToString_Response_<ContainerAllocator>;

  explicit StringToString_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->output = "";
    }
  }

  explicit StringToString_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : output(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->output = "";
    }
  }

  // field types and members
  using _output_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _output_type output;

  // setters for named parameter idiom
  Type & set__output(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->output = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface_pkg::srv::StringToString_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface_pkg::srv::StringToString_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface_pkg::srv::StringToString_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface_pkg::srv::StringToString_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface_pkg::srv::StringToString_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface_pkg::srv::StringToString_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface_pkg::srv::StringToString_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface_pkg::srv::StringToString_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface_pkg::srv::StringToString_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface_pkg::srv::StringToString_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface_pkg__srv__StringToString_Response
    std::shared_ptr<interface_pkg::srv::StringToString_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface_pkg__srv__StringToString_Response
    std::shared_ptr<interface_pkg::srv::StringToString_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StringToString_Response_ & other) const
  {
    if (this->output != other.output) {
      return false;
    }
    return true;
  }
  bool operator!=(const StringToString_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StringToString_Response_

// alias to use template instance with default allocator
using StringToString_Response =
  interface_pkg::srv::StringToString_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface_pkg


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__interface_pkg__srv__StringToString_Event __attribute__((deprecated))
#else
# define DEPRECATED__interface_pkg__srv__StringToString_Event __declspec(deprecated)
#endif

namespace interface_pkg
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StringToString_Event_
{
  using Type = StringToString_Event_<ContainerAllocator>;

  explicit StringToString_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit StringToString_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<interface_pkg::srv::StringToString_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<interface_pkg::srv::StringToString_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<interface_pkg::srv::StringToString_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<interface_pkg::srv::StringToString_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<interface_pkg::srv::StringToString_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<interface_pkg::srv::StringToString_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<interface_pkg::srv::StringToString_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<interface_pkg::srv::StringToString_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface_pkg::srv::StringToString_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface_pkg::srv::StringToString_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface_pkg::srv::StringToString_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface_pkg::srv::StringToString_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface_pkg::srv::StringToString_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface_pkg::srv::StringToString_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface_pkg::srv::StringToString_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface_pkg::srv::StringToString_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface_pkg::srv::StringToString_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface_pkg::srv::StringToString_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface_pkg__srv__StringToString_Event
    std::shared_ptr<interface_pkg::srv::StringToString_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface_pkg__srv__StringToString_Event
    std::shared_ptr<interface_pkg::srv::StringToString_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StringToString_Event_ & other) const
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
  bool operator!=(const StringToString_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StringToString_Event_

// alias to use template instance with default allocator
using StringToString_Event =
  interface_pkg::srv::StringToString_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface_pkg

namespace interface_pkg
{

namespace srv
{

struct StringToString
{
  using Request = interface_pkg::srv::StringToString_Request;
  using Response = interface_pkg::srv::StringToString_Response;
  using Event = interface_pkg::srv::StringToString_Event;
};

}  // namespace srv

}  // namespace interface_pkg

#endif  // INTERFACE_PKG__SRV__DETAIL__STRING_TO_STRING__STRUCT_HPP_
