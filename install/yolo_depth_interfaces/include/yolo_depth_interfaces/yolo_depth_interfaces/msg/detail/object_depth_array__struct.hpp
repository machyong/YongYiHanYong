// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yolo_depth_interfaces:msg/ObjectDepthArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "yolo_depth_interfaces/msg/object_depth_array.hpp"


#ifndef YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH_ARRAY__STRUCT_HPP_
#define YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'objects'
#include "yolo_depth_interfaces/msg/detail/object_depth__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__yolo_depth_interfaces__msg__ObjectDepthArray __attribute__((deprecated))
#else
# define DEPRECATED__yolo_depth_interfaces__msg__ObjectDepthArray __declspec(deprecated)
#endif

namespace yolo_depth_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectDepthArray_
{
  using Type = ObjectDepthArray_<ContainerAllocator>;

  explicit ObjectDepthArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit ObjectDepthArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _objects_type =
    std::vector<yolo_depth_interfaces::msg::ObjectDepth_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<yolo_depth_interfaces::msg::ObjectDepth_<ContainerAllocator>>>;
  _objects_type objects;

  // setters for named parameter idiom
  Type & set__objects(
    const std::vector<yolo_depth_interfaces::msg::ObjectDepth_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<yolo_depth_interfaces::msg::ObjectDepth_<ContainerAllocator>>> & _arg)
  {
    this->objects = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yolo_depth_interfaces::msg::ObjectDepthArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const yolo_depth_interfaces::msg::ObjectDepthArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yolo_depth_interfaces::msg::ObjectDepthArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yolo_depth_interfaces::msg::ObjectDepthArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yolo_depth_interfaces::msg::ObjectDepthArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yolo_depth_interfaces::msg::ObjectDepthArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yolo_depth_interfaces::msg::ObjectDepthArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yolo_depth_interfaces::msg::ObjectDepthArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yolo_depth_interfaces::msg::ObjectDepthArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yolo_depth_interfaces::msg::ObjectDepthArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yolo_depth_interfaces__msg__ObjectDepthArray
    std::shared_ptr<yolo_depth_interfaces::msg::ObjectDepthArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yolo_depth_interfaces__msg__ObjectDepthArray
    std::shared_ptr<yolo_depth_interfaces::msg::ObjectDepthArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectDepthArray_ & other) const
  {
    if (this->objects != other.objects) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectDepthArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectDepthArray_

// alias to use template instance with default allocator
using ObjectDepthArray =
  yolo_depth_interfaces::msg::ObjectDepthArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yolo_depth_interfaces

#endif  // YOLO_DEPTH_INTERFACES__MSG__DETAIL__OBJECT_DEPTH_ARRAY__STRUCT_HPP_
