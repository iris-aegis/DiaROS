// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Imm.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IMM__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__IMM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/imm__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Imm_mod
{
public:
  Init_Imm_mod()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::msg::Imm mod(::interfaces::msg::Imm::_mod_type arg)
  {
    msg_.mod = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Imm msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Imm>()
{
  return interfaces::msg::builder::Init_Imm_mod();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__IMM__BUILDER_HPP_
