// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Itt.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ITT__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__ITT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/itt__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Itt_confidence
{
public:
  explicit Init_Itt_confidence(::interfaces::msg::Itt & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Itt confidence(::interfaces::msg::Itt::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Itt msg_;
};

class Init_Itt_result
{
public:
  Init_Itt_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Itt_confidence result(::interfaces::msg::Itt::_result_type arg)
  {
    msg_.result = std::move(arg);
    return Init_Itt_confidence(msg_);
  }

private:
  ::interfaces::msg::Itt msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Itt>()
{
  return interfaces::msg::builder::Init_Itt_result();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__ITT__BUILDER_HPP_
