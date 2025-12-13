// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/Idm.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IDM__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__IDM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/idm__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Idm & msg,
  std::ostream & out)
{
  out << "{";
  // member: words
  {
    if (msg.words.size() == 0) {
      out << "words: []";
    } else {
      out << "words: [";
      size_t pending_items = msg.words.size();
      for (auto item : msg.words) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: session_id
  {
    out << "session_id: ";
    rosidl_generator_traits::value_to_yaml(msg.session_id, out);
    out << ", ";
  }

  // member: stage
  {
    out << "stage: ";
    rosidl_generator_traits::value_to_yaml(msg.stage, out);
    out << ", ";
  }

  // member: request_id
  {
    out << "request_id: ";
    rosidl_generator_traits::value_to_yaml(msg.request_id, out);
    out << ", ";
  }

  // member: turn_taking_decision_timestamp_ns
  {
    out << "turn_taking_decision_timestamp_ns: ";
    rosidl_generator_traits::value_to_yaml(msg.turn_taking_decision_timestamp_ns, out);
    out << ", ";
  }

  // member: first_stage_backchannel_at_tt
  {
    out << "first_stage_backchannel_at_tt: ";
    rosidl_generator_traits::value_to_yaml(msg.first_stage_backchannel_at_tt, out);
    out << ", ";
  }

  // member: asr_history_2_5s
  {
    if (msg.asr_history_2_5s.size() == 0) {
      out << "asr_history_2_5s: []";
    } else {
      out << "asr_history_2_5s: [";
      size_t pending_items = msg.asr_history_2_5s.size();
      for (auto item : msg.asr_history_2_5s) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Idm & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: words
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.words.size() == 0) {
      out << "words: []\n";
    } else {
      out << "words:\n";
      for (auto item : msg.words) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: session_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "session_id: ";
    rosidl_generator_traits::value_to_yaml(msg.session_id, out);
    out << "\n";
  }

  // member: stage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stage: ";
    rosidl_generator_traits::value_to_yaml(msg.stage, out);
    out << "\n";
  }

  // member: request_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "request_id: ";
    rosidl_generator_traits::value_to_yaml(msg.request_id, out);
    out << "\n";
  }

  // member: turn_taking_decision_timestamp_ns
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "turn_taking_decision_timestamp_ns: ";
    rosidl_generator_traits::value_to_yaml(msg.turn_taking_decision_timestamp_ns, out);
    out << "\n";
  }

  // member: first_stage_backchannel_at_tt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "first_stage_backchannel_at_tt: ";
    rosidl_generator_traits::value_to_yaml(msg.first_stage_backchannel_at_tt, out);
    out << "\n";
  }

  // member: asr_history_2_5s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.asr_history_2_5s.size() == 0) {
      out << "asr_history_2_5s: []\n";
    } else {
      out << "asr_history_2_5s:\n";
      for (auto item : msg.asr_history_2_5s) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Idm & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::msg::Idm & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::Idm & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::Idm>()
{
  return "interfaces::msg::Idm";
}

template<>
inline const char * name<interfaces::msg::Idm>()
{
  return "interfaces/msg/Idm";
}

template<>
struct has_fixed_size<interfaces::msg::Idm>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::msg::Idm>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::msg::Idm>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__IDM__TRAITS_HPP_
