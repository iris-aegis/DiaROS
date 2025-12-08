// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/Inlg.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__INLG__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__INLG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__Inlg __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__Inlg __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Inlg_
{
  using Type = Inlg_<ContainerAllocator>;

  explicit Inlg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->reply = "";
      this->stage = "";
      this->request_id = 0l;
      this->worker_name = "";
      this->start_timestamp_ns = 0ll;
      this->completion_timestamp_ns = 0ll;
      this->inference_duration_ms = 0.0;
      this->session_id = "";
    }
  }

  explicit Inlg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : reply(_alloc),
    stage(_alloc),
    worker_name(_alloc),
    session_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->reply = "";
      this->stage = "";
      this->request_id = 0l;
      this->worker_name = "";
      this->start_timestamp_ns = 0ll;
      this->completion_timestamp_ns = 0ll;
      this->inference_duration_ms = 0.0;
      this->session_id = "";
    }
  }

  // field types and members
  using _reply_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reply_type reply;
  using _source_words_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _source_words_type source_words;
  using _stage_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _stage_type stage;
  using _request_id_type =
    int32_t;
  _request_id_type request_id;
  using _worker_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _worker_name_type worker_name;
  using _start_timestamp_ns_type =
    int64_t;
  _start_timestamp_ns_type start_timestamp_ns;
  using _completion_timestamp_ns_type =
    int64_t;
  _completion_timestamp_ns_type completion_timestamp_ns;
  using _inference_duration_ms_type =
    double;
  _inference_duration_ms_type inference_duration_ms;
  using _session_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _session_id_type session_id;

  // setters for named parameter idiom
  Type & set__reply(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reply = _arg;
    return *this;
  }
  Type & set__source_words(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->source_words = _arg;
    return *this;
  }
  Type & set__stage(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->stage = _arg;
    return *this;
  }
  Type & set__request_id(
    const int32_t & _arg)
  {
    this->request_id = _arg;
    return *this;
  }
  Type & set__worker_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->worker_name = _arg;
    return *this;
  }
  Type & set__start_timestamp_ns(
    const int64_t & _arg)
  {
    this->start_timestamp_ns = _arg;
    return *this;
  }
  Type & set__completion_timestamp_ns(
    const int64_t & _arg)
  {
    this->completion_timestamp_ns = _arg;
    return *this;
  }
  Type & set__inference_duration_ms(
    const double & _arg)
  {
    this->inference_duration_ms = _arg;
    return *this;
  }
  Type & set__session_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->session_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::Inlg_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::Inlg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::Inlg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::Inlg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Inlg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Inlg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Inlg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Inlg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::Inlg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::Inlg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__Inlg
    std::shared_ptr<interfaces::msg::Inlg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__Inlg
    std::shared_ptr<interfaces::msg::Inlg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Inlg_ & other) const
  {
    if (this->reply != other.reply) {
      return false;
    }
    if (this->source_words != other.source_words) {
      return false;
    }
    if (this->stage != other.stage) {
      return false;
    }
    if (this->request_id != other.request_id) {
      return false;
    }
    if (this->worker_name != other.worker_name) {
      return false;
    }
    if (this->start_timestamp_ns != other.start_timestamp_ns) {
      return false;
    }
    if (this->completion_timestamp_ns != other.completion_timestamp_ns) {
      return false;
    }
    if (this->inference_duration_ms != other.inference_duration_ms) {
      return false;
    }
    if (this->session_id != other.session_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Inlg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Inlg_

// alias to use template instance with default allocator
using Inlg =
  interfaces::msg::Inlg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__INLG__STRUCT_HPP_
