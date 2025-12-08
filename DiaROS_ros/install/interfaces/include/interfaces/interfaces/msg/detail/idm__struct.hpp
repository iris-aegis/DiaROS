// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/Idm.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IDM__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__IDM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__Idm __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__Idm __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Idm_
{
  using Type = Idm_<ContainerAllocator>;

  explicit Idm_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->session_id = "";
      this->stage = "";
      this->turn_taking_decision_timestamp_ns = 0ll;
    }
  }

  explicit Idm_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : session_id(_alloc),
    stage(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->session_id = "";
      this->stage = "";
      this->turn_taking_decision_timestamp_ns = 0ll;
    }
  }

  // field types and members
  using _words_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _words_type words;
  using _session_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _session_id_type session_id;
  using _stage_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _stage_type stage;
  using _turn_taking_decision_timestamp_ns_type =
    int64_t;
  _turn_taking_decision_timestamp_ns_type turn_taking_decision_timestamp_ns;

  // setters for named parameter idiom
  Type & set__words(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->words = _arg;
    return *this;
  }
  Type & set__session_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->session_id = _arg;
    return *this;
  }
  Type & set__stage(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->stage = _arg;
    return *this;
  }
  Type & set__turn_taking_decision_timestamp_ns(
    const int64_t & _arg)
  {
    this->turn_taking_decision_timestamp_ns = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::Idm_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::Idm_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::Idm_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::Idm_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Idm_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Idm_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Idm_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Idm_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::Idm_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::Idm_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__Idm
    std::shared_ptr<interfaces::msg::Idm_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__Idm
    std::shared_ptr<interfaces::msg::Idm_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Idm_ & other) const
  {
    if (this->words != other.words) {
      return false;
    }
    if (this->session_id != other.session_id) {
      return false;
    }
    if (this->stage != other.stage) {
      return false;
    }
    if (this->turn_taking_decision_timestamp_ns != other.turn_taking_decision_timestamp_ns) {
      return false;
    }
    return true;
  }
  bool operator!=(const Idm_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Idm_

// alias to use template instance with default allocator
using Idm =
  interfaces::msg::Idm_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__IDM__STRUCT_HPP_
