// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/Iaa.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IAA__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__IAA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__Iaa __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__Iaa __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Iaa_
{
  using Type = Iaa_<ContainerAllocator>;

  explicit Iaa_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->f0 = 0.0f;
      this->grad = 0.0f;
      this->power = 0.0f;
      this->zerocross = 0l;
    }
  }

  explicit Iaa_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->f0 = 0.0f;
      this->grad = 0.0f;
      this->power = 0.0f;
      this->zerocross = 0l;
    }
  }

  // field types and members
  using _f0_type =
    float;
  _f0_type f0;
  using _grad_type =
    float;
  _grad_type grad;
  using _power_type =
    float;
  _power_type power;
  using _zerocross_type =
    int32_t;
  _zerocross_type zerocross;

  // setters for named parameter idiom
  Type & set__f0(
    const float & _arg)
  {
    this->f0 = _arg;
    return *this;
  }
  Type & set__grad(
    const float & _arg)
  {
    this->grad = _arg;
    return *this;
  }
  Type & set__power(
    const float & _arg)
  {
    this->power = _arg;
    return *this;
  }
  Type & set__zerocross(
    const int32_t & _arg)
  {
    this->zerocross = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::Iaa_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::Iaa_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::Iaa_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::Iaa_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Iaa_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Iaa_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Iaa_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Iaa_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::Iaa_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::Iaa_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__Iaa
    std::shared_ptr<interfaces::msg::Iaa_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__Iaa
    std::shared_ptr<interfaces::msg::Iaa_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Iaa_ & other) const
  {
    if (this->f0 != other.f0) {
      return false;
    }
    if (this->grad != other.grad) {
      return false;
    }
    if (this->power != other.power) {
      return false;
    }
    if (this->zerocross != other.zerocross) {
      return false;
    }
    return true;
  }
  bool operator!=(const Iaa_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Iaa_

// alias to use template instance with default allocator
using Iaa =
  interfaces::msg::Iaa_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__IAA__STRUCT_HPP_
