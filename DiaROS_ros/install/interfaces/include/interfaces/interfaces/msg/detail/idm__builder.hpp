// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Idm.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IDM__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__IDM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/idm__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Idm_words
{
public:
  Init_Idm_words()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::msg::Idm words(::interfaces::msg::Idm::_words_type arg)
  {
    msg_.words = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Idm msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Idm>()
{
  return interfaces::msg::builder::Init_Idm_words();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__IDM__BUILDER_HPP_
