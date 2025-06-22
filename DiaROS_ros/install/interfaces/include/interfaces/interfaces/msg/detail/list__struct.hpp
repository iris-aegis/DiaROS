// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/List.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__LIST__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__LIST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__List __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__List __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct List_
{
  using Type = List_<ContainerAllocator>;

  explicit List_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->n = 0ll;
      this->you = "";
      this->bot = "";
      this->frequency = 0.0f;
    }
  }

  explicit List_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : you(_alloc),
    bot(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->n = 0ll;
      this->you = "";
      this->bot = "";
      this->frequency = 0.0f;
    }
  }

  // field types and members
  using _n_type =
    int64_t;
  _n_type n;
  using _you_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _you_type you;
  using _bot_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _bot_type bot;
  using _frequency_type =
    float;
  _frequency_type frequency;

  // setters for named parameter idiom
  Type & set__n(
    const int64_t & _arg)
  {
    this->n = _arg;
    return *this;
  }
  Type & set__you(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->you = _arg;
    return *this;
  }
  Type & set__bot(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->bot = _arg;
    return *this;
  }
  Type & set__frequency(
    const float & _arg)
  {
    this->frequency = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::List_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::List_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::List_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::List_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::List_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::List_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::List_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::List_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::List_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::List_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__List
    std::shared_ptr<interfaces::msg::List_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__List
    std::shared_ptr<interfaces::msg::List_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const List_ & other) const
  {
    if (this->n != other.n) {
      return false;
    }
    if (this->you != other.you) {
      return false;
    }
    if (this->bot != other.bot) {
      return false;
    }
    if (this->frequency != other.frequency) {
      return false;
    }
    return true;
  }
  bool operator!=(const List_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct List_

// alias to use template instance with default allocator
using List =
  interfaces::msg::List_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__LIST__STRUCT_HPP_
