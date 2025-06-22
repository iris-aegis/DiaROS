// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/Imm.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IMM__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__IMM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__Imm __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__Imm __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Imm_
{
  using Type = Imm_<ContainerAllocator>;

  explicit Imm_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mod = "";
    }
  }

  explicit Imm_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : mod(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mod = "";
    }
  }

  // field types and members
  using _mod_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mod_type mod;

  // setters for named parameter idiom
  Type & set__mod(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mod = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::Imm_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::Imm_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::Imm_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::Imm_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Imm_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Imm_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Imm_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Imm_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::Imm_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::Imm_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__Imm
    std::shared_ptr<interfaces::msg::Imm_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__Imm
    std::shared_ptr<interfaces::msg::Imm_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Imm_ & other) const
  {
    if (this->mod != other.mod) {
      return false;
    }
    return true;
  }
  bool operator!=(const Imm_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Imm_

// alias to use template instance with default allocator
using Imm =
  interfaces::msg::Imm_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__IMM__STRUCT_HPP_
