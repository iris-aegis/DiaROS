// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/Iasr.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IASR__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__IASR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/iasr__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Iasr & msg,
  std::ostream & out)
{
  out << "{";
  // member: you
  {
    out << "you: ";
    rosidl_generator_traits::value_to_yaml(msg.you, out);
    out << ", ";
  }

  // member: is_final
  {
    out << "is_final: ";
    rosidl_generator_traits::value_to_yaml(msg.is_final, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Iasr & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: you
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "you: ";
    rosidl_generator_traits::value_to_yaml(msg.you, out);
    out << "\n";
  }

  // member: is_final
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_final: ";
    rosidl_generator_traits::value_to_yaml(msg.is_final, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Iasr & msg, bool use_flow_style = false)
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
  const interfaces::msg::Iasr & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::Iasr & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::Iasr>()
{
  return "interfaces::msg::Iasr";
}

template<>
inline const char * name<interfaces::msg::Iasr>()
{
  return "interfaces/msg/Iasr";
}

template<>
struct has_fixed_size<interfaces::msg::Iasr>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::msg::Iasr>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::msg::Iasr>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__IASR__TRAITS_HPP_
