// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/Itt.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ITT__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__ITT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__Itt __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__Itt __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Itt_
{
  using Type = Itt_<ContainerAllocator>;

  explicit Itt_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
      this->confidence = 0.0f;
    }
  }

  explicit Itt_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    interfaces::msg::Itt_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::Itt_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::Itt_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::Itt_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Itt_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Itt_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Itt_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Itt_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::Itt_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::Itt_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__Itt
    std::shared_ptr<interfaces::msg::Itt_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__Itt
    std::shared_ptr<interfaces::msg::Itt_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Itt_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    return true;
  }
  bool operator!=(const Itt_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Itt_

// alias to use template instance with default allocator
using Itt =
  interfaces::msg::Itt_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__ITT__STRUCT_HPP_
