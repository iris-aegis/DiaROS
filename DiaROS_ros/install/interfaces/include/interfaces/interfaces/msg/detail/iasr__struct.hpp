// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/Iasr.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IASR__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__IASR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__Iasr __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__Iasr __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Iasr_
{
  using Type = Iasr_<ContainerAllocator>;

  explicit Iasr_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->you = "";
      this->is_final = false;
    }
  }

  explicit Iasr_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : you(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->you = "";
      this->is_final = false;
    }
  }

  // field types and members
  using _you_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _you_type you;
  using _is_final_type =
    bool;
  _is_final_type is_final;

  // setters for named parameter idiom
  Type & set__you(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->you = _arg;
    return *this;
  }
  Type & set__is_final(
    const bool & _arg)
  {
    this->is_final = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::Iasr_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::Iasr_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::Iasr_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::Iasr_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Iasr_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Iasr_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Iasr_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Iasr_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::Iasr_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::Iasr_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__Iasr
    std::shared_ptr<interfaces::msg::Iasr_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__Iasr
    std::shared_ptr<interfaces::msg::Iasr_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Iasr_ & other) const
  {
    if (this->you != other.you) {
      return false;
    }
    if (this->is_final != other.is_final) {
      return false;
    }
    return true;
  }
  bool operator!=(const Iasr_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Iasr_

// alias to use template instance with default allocator
using Iasr =
  interfaces::msg::Iasr_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__IASR__STRUCT_HPP_
