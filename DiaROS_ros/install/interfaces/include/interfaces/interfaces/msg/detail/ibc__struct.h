// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/Ibc.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IBC__STRUCT_H_
#define INTERFACES__MSG__DETAIL__IBC__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Ibc in the package interfaces.
typedef struct interfaces__msg__Ibc
{
  int8_t result;
  float confidence;
} interfaces__msg__Ibc;

// Struct for a sequence of interfaces__msg__Ibc.
typedef struct interfaces__msg__Ibc__Sequence
{
  interfaces__msg__Ibc * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__Ibc__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__IBC__STRUCT_H_
