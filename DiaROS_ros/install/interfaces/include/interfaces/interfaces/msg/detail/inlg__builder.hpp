// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Inlg.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__INLG__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__INLG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/inlg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Inlg_reply
{
public:
  Init_Inlg_reply()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::msg::Inlg reply(::interfaces::msg::Inlg::_reply_type arg)
  {
    msg_.reply = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Inlg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Inlg>()
{
  return interfaces::msg::builder::Init_Inlg_reply();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__INLG__BUILDER_HPP_
