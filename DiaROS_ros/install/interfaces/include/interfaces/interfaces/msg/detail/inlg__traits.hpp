// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/Inlg.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__INLG__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__INLG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/inlg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Inlg & msg,
  std::ostream & out)
{
  out << "{";
  // member: reply
  {
    out << "reply: ";
    rosidl_generator_traits::value_to_yaml(msg.reply, out);
    out << ", ";
  }

  // member: source_words
  {
    if (msg.source_words.size() == 0) {
      out << "source_words: []";
    } else {
      out << "source_words: [";
      size_t pending_items = msg.source_words.size();
      for (auto item : msg.source_words) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: request_id
  {
    out << "request_id: ";
    rosidl_generator_traits::value_to_yaml(msg.request_id, out);
    out << ", ";
  }

  // member: worker_name
  {
    out << "worker_name: ";
    rosidl_generator_traits::value_to_yaml(msg.worker_name, out);
    out << ", ";
  }

  // member: start_timestamp_ns
  {
    out << "start_timestamp_ns: ";
    rosidl_generator_traits::value_to_yaml(msg.start_timestamp_ns, out);
    out << ", ";
  }

  // member: completion_timestamp_ns
  {
    out << "completion_timestamp_ns: ";
    rosidl_generator_traits::value_to_yaml(msg.completion_timestamp_ns, out);
    out << ", ";
  }

  // member: inference_duration_ms
  {
    out << "inference_duration_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.inference_duration_ms, out);
    out << ", ";
  }

  // member: session_id
  {
    out << "session_id: ";
    rosidl_generator_traits::value_to_yaml(msg.session_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Inlg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: reply
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reply: ";
    rosidl_generator_traits::value_to_yaml(msg.reply, out);
    out << "\n";
  }

  // member: source_words
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.source_words.size() == 0) {
      out << "source_words: []\n";
    } else {
      out << "source_words:\n";
      for (auto item : msg.source_words) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
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

  // member: worker_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "worker_name: ";
    rosidl_generator_traits::value_to_yaml(msg.worker_name, out);
    out << "\n";
  }

  // member: start_timestamp_ns
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_timestamp_ns: ";
    rosidl_generator_traits::value_to_yaml(msg.start_timestamp_ns, out);
    out << "\n";
  }

  // member: completion_timestamp_ns
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "completion_timestamp_ns: ";
    rosidl_generator_traits::value_to_yaml(msg.completion_timestamp_ns, out);
    out << "\n";
  }

  // member: inference_duration_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "inference_duration_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.inference_duration_ms, out);
    out << "\n";
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Inlg & msg, bool use_flow_style = false)
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
  const interfaces::msg::Inlg & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::Inlg & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::Inlg>()
{
  return "interfaces::msg::Inlg";
}

template<>
inline const char * name<interfaces::msg::Inlg>()
{
  return "interfaces/msg/Inlg";
}

template<>
struct has_fixed_size<interfaces::msg::Inlg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::msg::Inlg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::msg::Inlg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__INLG__TRAITS_HPP_
