// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Iaa.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IAA__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__IAA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/iaa__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Iaa_zerocross
{
public:
  explicit Init_Iaa_zerocross(::interfaces::msg::Iaa & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Iaa zerocross(::interfaces::msg::Iaa::_zerocross_type arg)
  {
    msg_.zerocross = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Iaa msg_;
};

class Init_Iaa_power
{
public:
  explicit Init_Iaa_power(::interfaces::msg::Iaa & msg)
  : msg_(msg)
  {}
  Init_Iaa_zerocross power(::interfaces::msg::Iaa::_power_type arg)
  {
    msg_.power = std::move(arg);
    return Init_Iaa_zerocross(msg_);
  }

private:
  ::interfaces::msg::Iaa msg_;
};

class Init_Iaa_grad
{
public:
  explicit Init_Iaa_grad(::interfaces::msg::Iaa & msg)
  : msg_(msg)
  {}
  Init_Iaa_power grad(::interfaces::msg::Iaa::_grad_type arg)
  {
    msg_.grad = std::move(arg);
    return Init_Iaa_power(msg_);
  }

private:
  ::interfaces::msg::Iaa msg_;
};

class Init_Iaa_f0
{
public:
  Init_Iaa_f0()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Iaa_grad f0(::interfaces::msg::Iaa::_f0_type arg)
  {
    msg_.f0 = std::move(arg);
    return Init_Iaa_grad(msg_);
  }

private:
  ::interfaces::msg::Iaa msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Iaa>()
{
  return interfaces::msg::builder::Init_Iaa_f0();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__IAA__BUILDER_HPP_
