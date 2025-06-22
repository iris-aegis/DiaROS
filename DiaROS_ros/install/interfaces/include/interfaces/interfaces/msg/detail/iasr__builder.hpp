// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Iasr.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IASR__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__IASR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/iasr__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Iasr_is_final
{
public:
  explicit Init_Iasr_is_final(::interfaces::msg::Iasr & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Iasr is_final(::interfaces::msg::Iasr::_is_final_type arg)
  {
    msg_.is_final = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Iasr msg_;
};

class Init_Iasr_you
{
public:
  Init_Iasr_you()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Iasr_is_final you(::interfaces::msg::Iasr::_you_type arg)
  {
    msg_.you = std::move(arg);
    return Init_Iasr_is_final(msg_);
  }

private:
  ::interfaces::msg::Iasr msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Iasr>()
{
  return interfaces::msg::builder::Init_Iasr_you();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__IASR__BUILDER_HPP_
