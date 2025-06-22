// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/Ibc.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IBC__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__IBC__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__Ibc __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__Ibc __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Ibc_
{
  using Type = Ibc_<ContainerAllocator>;

  explicit Ibc_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
      this->confidence = 0.0f;
    }
  }

  explicit Ibc_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
      this->confidence = 0.0f;
    }
  }

  // field types and members
  using _result_type =
    int8_t;
  _result_type result;
  using _confidence_type =
    float;
  _confidence_type confidence;

  // setters for named parameter idiom
  Type & set__result(
    const int8_t & _arg)
  {
    this->result = _arg;
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
    interfaces::msg::Ibc_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::Ibc_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::Ibc_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::Ibc_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Ibc_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Ibc_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Ibc_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Ibc_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::Ibc_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::Ibc_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__Ibc
    std::shared_ptr<interfaces::msg::Ibc_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__Ibc
    std::shared_ptr<interfaces::msg::Ibc_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Ibc_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    return true;
  }
  bool operator!=(const Ibc_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Ibc_

// alias to use template instance with default allocator
using Ibc =
  interfaces::msg::Ibc_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__IBC__STRUCT_HPP_
