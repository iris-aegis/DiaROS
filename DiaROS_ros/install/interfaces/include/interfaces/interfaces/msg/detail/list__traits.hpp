// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/List.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__LIST__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const List & msg,
  std::ostream & out)
{
  out << "{";
  // member: n
  {
    out << "n: ";
    rosidl_generator_traits::value_to_yaml(msg.n, out);
    out << ", ";
  }

  // member: you
  {
    out << "you: ";
    rosidl_generator_traits::value_to_yaml(msg.you, out);
    out << ", ";
  }

  // member: bot
  {
    out << "bot: ";
    rosidl_generator_traits::value_to_yaml(msg.bot, out);
    out << ", ";
  }

  // member: frequency
  {
    out << "frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.frequency, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const List & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: n
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "n: ";
    rosidl_generator_traits::value_to_yaml(msg.n, out);
    out << "\n";
  }

  // member: you
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "you: ";
    rosidl_generator_traits::value_to_yaml(msg.you, out);
    out << "\n";
  }

  // member: bot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bot: ";
    rosidl_generator_traits::value_to_yaml(msg.bot, out);
    out << "\n";
  }

  // member: frequency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.frequency, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const List & msg, bool use_flow_style = false)
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
  const interfaces::msg::List & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::List & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::List>()
{
  return "interfaces::msg::List";
}

template<>
inline const char * name<interfaces::msg::List>()
{
  return "interfaces/msg/List";
}

template<>
struct has_fixed_size<interfaces::msg::List>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::msg::List>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::msg::List>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__LIST__TRAITS_HPP_
