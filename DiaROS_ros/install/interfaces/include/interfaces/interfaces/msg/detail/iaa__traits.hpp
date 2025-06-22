// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/Iaa.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IAA__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__IAA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/iaa__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Iaa & msg,
  std::ostream & out)
{
  out << "{";
  // member: f0
  {
    out << "f0: ";
    rosidl_generator_traits::value_to_yaml(msg.f0, out);
    out << ", ";
  }

  // member: grad
  {
    out << "grad: ";
    rosidl_generator_traits::value_to_yaml(msg.grad, out);
    out << ", ";
  }

  // member: power
  {
    out << "power: ";
    rosidl_generator_traits::value_to_yaml(msg.power, out);
    out << ", ";
  }

  // member: zerocross
  {
    out << "zerocross: ";
    rosidl_generator_traits::value_to_yaml(msg.zerocross, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Iaa & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: f0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "f0: ";
    rosidl_generator_traits::value_to_yaml(msg.f0, out);
    out << "\n";
  }

  // member: grad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "grad: ";
    rosidl_generator_traits::value_to_yaml(msg.grad, out);
    out << "\n";
  }

  // member: power
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "power: ";
    rosidl_generator_traits::value_to_yaml(msg.power, out);
    out << "\n";
  }

  // member: zerocross
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "zerocross: ";
    rosidl_generator_traits::value_to_yaml(msg.zerocross, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Iaa & msg, bool use_flow_style = false)
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
  const interfaces::msg::Iaa & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::Iaa & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::Iaa>()
{
  return "interfaces::msg::Iaa";
}

template<>
inline const char * name<interfaces::msg::Iaa>()
{
  return "interfaces/msg/Iaa";
}

template<>
struct has_fixed_size<interfaces::msg::Iaa>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interfaces::msg::Iaa>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interfaces::msg::Iaa>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__IAA__TRAITS_HPP_
