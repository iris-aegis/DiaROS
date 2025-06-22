// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Ibc.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IBC__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__IBC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/ibc__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Ibc_confidence
{
public:
  explicit Init_Ibc_confidence(::interfaces::msg::Ibc & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Ibc confidence(::interfaces::msg::Ibc::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Ibc msg_;
};

class Init_Ibc_result
{
public:
  Init_Ibc_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Ibc_confidence result(::interfaces::msg::Ibc::_result_type arg)
  {
    msg_.result = std::move(arg);
    return Init_Ibc_confidence(msg_);
  }

private:
  ::interfaces::msg::Ibc msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Ibc>()
{
  return interfaces::msg::builder::Init_Ibc_result();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__IBC__BUILDER_HPP_
