// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/List.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__LIST__STRUCT_H_
#define INTERFACES__MSG__DETAIL__LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'you'
// Member 'bot'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/List in the package interfaces.
typedef struct interfaces__msg__List
{
  int64_t n;
  rosidl_runtime_c__String you;
  rosidl_runtime_c__String bot;
  float frequency;
} interfaces__msg__List;

// Struct for a sequence of interfaces__msg__List.
typedef struct interfaces__msg__List__Sequence
{
  interfaces__msg__List * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__List__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__LIST__STRUCT_H_
