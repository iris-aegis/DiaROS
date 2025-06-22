// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/Iaa.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IAA__STRUCT_H_
#define INTERFACES__MSG__DETAIL__IAA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Iaa in the package interfaces.
typedef struct interfaces__msg__Iaa
{
  float f0;
  float grad;
  float power;
  int32_t zerocross;
} interfaces__msg__Iaa;

// Struct for a sequence of interfaces__msg__Iaa.
typedef struct interfaces__msg__Iaa__Sequence
{
  interfaces__msg__Iaa * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__Iaa__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__IAA__STRUCT_H_
