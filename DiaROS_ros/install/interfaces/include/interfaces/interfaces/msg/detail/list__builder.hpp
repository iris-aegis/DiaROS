// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/List.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__LIST__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_List_frequency
{
public:
  explicit Init_List_frequency(::interfaces::msg::List & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::List frequency(::interfaces::msg::List::_frequency_type arg)
  {
    msg_.frequency = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::List msg_;
};

class Init_List_bot
{
public:
  explicit Init_List_bot(::interfaces::msg::List & msg)
  : msg_(msg)
  {}
  Init_List_frequency bot(::interfaces::msg::List::_bot_type arg)
  {
    msg_.bot = std::move(arg);
    return Init_List_frequency(msg_);
  }

private:
  ::interfaces::msg::List msg_;
};

class Init_List_you
{
public:
  explicit Init_List_you(::interfaces::msg::List & msg)
  : msg_(msg)
  {}
  Init_List_bot you(::interfaces::msg::List::_you_type arg)
  {
    msg_.you = std::move(arg);
    return Init_List_bot(msg_);
  }

private:
  ::interfaces::msg::List msg_;
};

class Init_List_n
{
public:
  Init_List_n()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_List_you n(::interfaces::msg::List::_n_type arg)
  {
    msg_.n = std::move(arg);
    return Init_List_you(msg_);
  }

private:
  ::interfaces::msg::List msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::List>()
{
  return interfaces::msg::builder::Init_List_n();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__LIST__BUILDER_HPP_
