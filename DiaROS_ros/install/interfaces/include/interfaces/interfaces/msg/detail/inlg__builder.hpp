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

class Init_Inlg_session_id
{
public:
  explicit Init_Inlg_session_id(::interfaces::msg::Inlg & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Inlg session_id(::interfaces::msg::Inlg::_session_id_type arg)
  {
    msg_.session_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Inlg msg_;
};

class Init_Inlg_inference_duration_ms
{
public:
  explicit Init_Inlg_inference_duration_ms(::interfaces::msg::Inlg & msg)
  : msg_(msg)
  {}
  Init_Inlg_session_id inference_duration_ms(::interfaces::msg::Inlg::_inference_duration_ms_type arg)
  {
    msg_.inference_duration_ms = std::move(arg);
    return Init_Inlg_session_id(msg_);
  }

private:
  ::interfaces::msg::Inlg msg_;
};

class Init_Inlg_completion_timestamp_ns
{
public:
  explicit Init_Inlg_completion_timestamp_ns(::interfaces::msg::Inlg & msg)
  : msg_(msg)
  {}
  Init_Inlg_inference_duration_ms completion_timestamp_ns(::interfaces::msg::Inlg::_completion_timestamp_ns_type arg)
  {
    msg_.completion_timestamp_ns = std::move(arg);
    return Init_Inlg_inference_duration_ms(msg_);
  }

private:
  ::interfaces::msg::Inlg msg_;
};

class Init_Inlg_start_timestamp_ns
{
public:
  explicit Init_Inlg_start_timestamp_ns(::interfaces::msg::Inlg & msg)
  : msg_(msg)
  {}
  Init_Inlg_completion_timestamp_ns start_timestamp_ns(::interfaces::msg::Inlg::_start_timestamp_ns_type arg)
  {
    msg_.start_timestamp_ns = std::move(arg);
    return Init_Inlg_completion_timestamp_ns(msg_);
  }

private:
  ::interfaces::msg::Inlg msg_;
};

class Init_Inlg_worker_name
{
public:
  explicit Init_Inlg_worker_name(::interfaces::msg::Inlg & msg)
  : msg_(msg)
  {}
  Init_Inlg_start_timestamp_ns worker_name(::interfaces::msg::Inlg::_worker_name_type arg)
  {
    msg_.worker_name = std::move(arg);
    return Init_Inlg_start_timestamp_ns(msg_);
  }

private:
  ::interfaces::msg::Inlg msg_;
};

class Init_Inlg_request_id
{
public:
  explicit Init_Inlg_request_id(::interfaces::msg::Inlg & msg)
  : msg_(msg)
  {}
  Init_Inlg_worker_name request_id(::interfaces::msg::Inlg::_request_id_type arg)
  {
    msg_.request_id = std::move(arg);
    return Init_Inlg_worker_name(msg_);
  }

private:
  ::interfaces::msg::Inlg msg_;
};

class Init_Inlg_stage
{
public:
  explicit Init_Inlg_stage(::interfaces::msg::Inlg & msg)
  : msg_(msg)
  {}
  Init_Inlg_request_id stage(::interfaces::msg::Inlg::_stage_type arg)
  {
    msg_.stage = std::move(arg);
    return Init_Inlg_request_id(msg_);
  }

private:
  ::interfaces::msg::Inlg msg_;
};

class Init_Inlg_source_words
{
public:
  explicit Init_Inlg_source_words(::interfaces::msg::Inlg & msg)
  : msg_(msg)
  {}
  Init_Inlg_stage source_words(::interfaces::msg::Inlg::_source_words_type arg)
  {
    msg_.source_words = std::move(arg);
    return Init_Inlg_stage(msg_);
  }

private:
  ::interfaces::msg::Inlg msg_;
};

class Init_Inlg_reply
{
public:
  Init_Inlg_reply()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Inlg_source_words reply(::interfaces::msg::Inlg::_reply_type arg)
  {
    msg_.reply = std::move(arg);
    return Init_Inlg_source_words(msg_);
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
