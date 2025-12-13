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

class Init_Idm_asr_history_2_5s
{
public:
  explicit Init_Idm_asr_history_2_5s(::interfaces::msg::Idm & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Idm asr_history_2_5s(::interfaces::msg::Idm::_asr_history_2_5s_type arg)
  {
    msg_.asr_history_2_5s = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Idm msg_;
};

class Init_Idm_first_stage_backchannel_at_tt
{
public:
  explicit Init_Idm_first_stage_backchannel_at_tt(::interfaces::msg::Idm & msg)
  : msg_(msg)
  {}
  Init_Idm_asr_history_2_5s first_stage_backchannel_at_tt(::interfaces::msg::Idm::_first_stage_backchannel_at_tt_type arg)
  {
    msg_.first_stage_backchannel_at_tt = std::move(arg);
    return Init_Idm_asr_history_2_5s(msg_);
  }

private:
  ::interfaces::msg::Idm msg_;
};

class Init_Idm_turn_taking_decision_timestamp_ns
{
public:
  explicit Init_Idm_turn_taking_decision_timestamp_ns(::interfaces::msg::Idm & msg)
  : msg_(msg)
  {}
  Init_Idm_first_stage_backchannel_at_tt turn_taking_decision_timestamp_ns(::interfaces::msg::Idm::_turn_taking_decision_timestamp_ns_type arg)
  {
    msg_.turn_taking_decision_timestamp_ns = std::move(arg);
    return Init_Idm_first_stage_backchannel_at_tt(msg_);
  }

private:
  ::interfaces::msg::Idm msg_;
};

class Init_Idm_request_id
{
public:
  explicit Init_Idm_request_id(::interfaces::msg::Idm & msg)
  : msg_(msg)
  {}
  Init_Idm_turn_taking_decision_timestamp_ns request_id(::interfaces::msg::Idm::_request_id_type arg)
  {
    msg_.request_id = std::move(arg);
    return Init_Idm_turn_taking_decision_timestamp_ns(msg_);
  }

private:
  ::interfaces::msg::Idm msg_;
};

class Init_Idm_stage
{
public:
  explicit Init_Idm_stage(::interfaces::msg::Idm & msg)
  : msg_(msg)
  {}
  Init_Idm_request_id stage(::interfaces::msg::Idm::_stage_type arg)
  {
    msg_.stage = std::move(arg);
    return Init_Idm_request_id(msg_);
  }

private:
  ::interfaces::msg::Idm msg_;
};

class Init_Idm_session_id
{
public:
  explicit Init_Idm_session_id(::interfaces::msg::Idm & msg)
  : msg_(msg)
  {}
  Init_Idm_stage session_id(::interfaces::msg::Idm::_session_id_type arg)
  {
    msg_.session_id = std::move(arg);
    return Init_Idm_stage(msg_);
  }

private:
  ::interfaces::msg::Idm msg_;
};

class Init_Idm_words
{
public:
  Init_Idm_words()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Idm_session_id words(::interfaces::msg::Idm::_words_type arg)
  {
    msg_.words = std::move(arg);
    return Init_Idm_session_id(msg_);
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
