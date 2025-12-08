// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from interfaces:msg/Inlg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "interfaces/msg/detail/inlg__rosidl_typesupport_introspection_c.h"
#include "interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "interfaces/msg/detail/inlg__functions.h"
#include "interfaces/msg/detail/inlg__struct.h"


// Include directives for member types
// Member `reply`
// Member `source_words`
// Member `stage`
// Member `worker_name`
// Member `session_id`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void interfaces__msg__Inlg__rosidl_typesupport_introspection_c__Inlg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interfaces__msg__Inlg__init(message_memory);
}

void interfaces__msg__Inlg__rosidl_typesupport_introspection_c__Inlg_fini_function(void * message_memory)
{
  interfaces__msg__Inlg__fini(message_memory);
}

size_t interfaces__msg__Inlg__rosidl_typesupport_introspection_c__size_function__Inlg__source_words(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * interfaces__msg__Inlg__rosidl_typesupport_introspection_c__get_const_function__Inlg__source_words(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interfaces__msg__Inlg__rosidl_typesupport_introspection_c__get_function__Inlg__source_words(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void interfaces__msg__Inlg__rosidl_typesupport_introspection_c__fetch_function__Inlg__source_words(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    interfaces__msg__Inlg__rosidl_typesupport_introspection_c__get_const_function__Inlg__source_words(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void interfaces__msg__Inlg__rosidl_typesupport_introspection_c__assign_function__Inlg__source_words(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    interfaces__msg__Inlg__rosidl_typesupport_introspection_c__get_function__Inlg__source_words(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool interfaces__msg__Inlg__rosidl_typesupport_introspection_c__resize_function__Inlg__source_words(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember interfaces__msg__Inlg__rosidl_typesupport_introspection_c__Inlg_message_member_array[9] = {
  {
    "reply",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__Inlg, reply),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "source_words",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__Inlg, source_words),  // bytes offset in struct
    NULL,  // default value
    interfaces__msg__Inlg__rosidl_typesupport_introspection_c__size_function__Inlg__source_words,  // size() function pointer
    interfaces__msg__Inlg__rosidl_typesupport_introspection_c__get_const_function__Inlg__source_words,  // get_const(index) function pointer
    interfaces__msg__Inlg__rosidl_typesupport_introspection_c__get_function__Inlg__source_words,  // get(index) function pointer
    interfaces__msg__Inlg__rosidl_typesupport_introspection_c__fetch_function__Inlg__source_words,  // fetch(index, &value) function pointer
    interfaces__msg__Inlg__rosidl_typesupport_introspection_c__assign_function__Inlg__source_words,  // assign(index, value) function pointer
    interfaces__msg__Inlg__rosidl_typesupport_introspection_c__resize_function__Inlg__source_words  // resize(index) function pointer
  },
  {
    "stage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__Inlg, stage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__Inlg, request_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "worker_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__Inlg, worker_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "start_timestamp_ns",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__Inlg, start_timestamp_ns),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "completion_timestamp_ns",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__Inlg, completion_timestamp_ns),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "inference_duration_ms",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__Inlg, inference_duration_ms),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "session_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__Inlg, session_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers interfaces__msg__Inlg__rosidl_typesupport_introspection_c__Inlg_message_members = {
  "interfaces__msg",  // message namespace
  "Inlg",  // message name
  9,  // number of fields
  sizeof(interfaces__msg__Inlg),
  interfaces__msg__Inlg__rosidl_typesupport_introspection_c__Inlg_message_member_array,  // message members
  interfaces__msg__Inlg__rosidl_typesupport_introspection_c__Inlg_init_function,  // function to initialize message memory (memory has to be allocated)
  interfaces__msg__Inlg__rosidl_typesupport_introspection_c__Inlg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t interfaces__msg__Inlg__rosidl_typesupport_introspection_c__Inlg_message_type_support_handle = {
  0,
  &interfaces__msg__Inlg__rosidl_typesupport_introspection_c__Inlg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interfaces, msg, Inlg)() {
  if (!interfaces__msg__Inlg__rosidl_typesupport_introspection_c__Inlg_message_type_support_handle.typesupport_identifier) {
    interfaces__msg__Inlg__rosidl_typesupport_introspection_c__Inlg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &interfaces__msg__Inlg__rosidl_typesupport_introspection_c__Inlg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
