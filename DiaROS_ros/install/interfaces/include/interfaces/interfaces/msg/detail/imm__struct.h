// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/Imm.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IMM__STRUCT_H_
#define INTERFACES__MSG__DETAIL__IMM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'mod'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Imm in the package interfaces.
typedef struct interfaces__msg__Imm
{
  rosidl_runtime_c__String mod;
} interfaces__msg__Imm;

// Struct for a sequence of interfaces__msg__Imm.
typedef struct interfaces__msg__Imm__Sequence
{
  interfaces__msg__Imm * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__Imm__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__IMM__STRUCT_H_
