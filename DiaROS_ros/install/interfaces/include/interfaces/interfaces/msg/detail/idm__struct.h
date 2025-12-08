// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/Idm.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IDM__STRUCT_H_
#define INTERFACES__MSG__DETAIL__IDM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'words'
// Member 'session_id'
// Member 'stage'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Idm in the package interfaces.
typedef struct interfaces__msg__Idm
{
  rosidl_runtime_c__String__Sequence words;
  rosidl_runtime_c__String session_id;
  rosidl_runtime_c__String stage;
  int64_t turn_taking_decision_timestamp_ns;
} interfaces__msg__Idm;

// Struct for a sequence of interfaces__msg__Idm.
typedef struct interfaces__msg__Idm__Sequence
{
  interfaces__msg__Idm * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__Idm__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__IDM__STRUCT_H_
