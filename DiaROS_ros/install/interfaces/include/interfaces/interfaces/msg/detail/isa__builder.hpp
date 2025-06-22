// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Isa.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ISA__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__ISA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/isa__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Isa_zerocross
{
public:
  explicit Init_Isa_zerocross(::interfaces::msg::Isa & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Isa zerocross(::interfaces::msg::Isa::_zerocross_type arg)
  {
    msg_.zerocross = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Isa msg_;
};

class Init_Isa_power
{
public:
  explicit Init_Isa_power(::interfaces::msg::Isa & msg)
  : msg_(msg)
  {}
  Init_Isa_zerocross power(::interfaces::msg::Isa::_power_type arg)
  {
    msg_.power = std::move(arg);
    return Init_Isa_zerocross(msg_);
  }

private:
  ::interfaces::msg::Isa msg_;
};

class Init_Isa_grad
{
public:
  explicit Init_Isa_grad(::interfaces::msg::Isa & msg)
  : msg_(msg)
  {}
  Init_Isa_power grad(::interfaces::msg::Isa::_grad_type arg)
  {
    msg_.grad = std::move(arg);
    return Init_Isa_power(msg_);
  }

private:
  ::interfaces::msg::Isa msg_;
};

class Init_Isa_frequency
{
public:
  explicit Init_Isa_frequency(::interfaces::msg::Isa & msg)
  : msg_(msg)
  {}
  Init_Isa_grad frequency(::interfaces::msg::Isa::_frequency_type arg)
  {
    msg_.frequency = std::move(arg);
    return Init_Isa_grad(msg_);
  }

private:
  ::interfaces::msg::Isa msg_;
};

class Init_Isa_prevgrad
{
public:
  Init_Isa_prevgrad()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Isa_frequency prevgrad(::interfaces::msg::Isa::_prevgrad_type arg)
  {
    msg_.prevgrad = std::move(arg);
    return Init_Isa_frequency(msg_);
  }

private:
  ::interfaces::msg::Isa msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Isa>()
{
  return interfaces::msg::builder::Init_Isa_prevgrad();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__ISA__BUILDER_HPP_
