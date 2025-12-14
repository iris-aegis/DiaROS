// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces:msg/Inlg.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/inlg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `reply`
// Member `stage`
// Member `source_words`
// Member `worker_name`
// Member `session_id`
#include "rosidl_runtime_c/string_functions.h"

bool
interfaces__msg__Inlg__init(interfaces__msg__Inlg * msg)
{
  if (!msg) {
    return false;
  }
  // reply
  if (!rosidl_runtime_c__String__init(&msg->reply)) {
    interfaces__msg__Inlg__fini(msg);
    return false;
  }
  // stage
  if (!rosidl_runtime_c__String__init(&msg->stage)) {
    interfaces__msg__Inlg__fini(msg);
    return false;
  }
  // source_words
  if (!rosidl_runtime_c__String__Sequence__init(&msg->source_words, 0)) {
    interfaces__msg__Inlg__fini(msg);
    return false;
  }
  // request_id
  // worker_name
  if (!rosidl_runtime_c__String__init(&msg->worker_name)) {
    interfaces__msg__Inlg__fini(msg);
    return false;
  }
  // start_timestamp_ns
  // completion_timestamp_ns
  // inference_duration_ms
  // session_id
  if (!rosidl_runtime_c__String__init(&msg->session_id)) {
    interfaces__msg__Inlg__fini(msg);
    return false;
  }
  return true;
}

void
interfaces__msg__Inlg__fini(interfaces__msg__Inlg * msg)
{
  if (!msg) {
    return;
  }
  // reply
  rosidl_runtime_c__String__fini(&msg->reply);
  // stage
  rosidl_runtime_c__String__fini(&msg->stage);
  // source_words
  rosidl_runtime_c__String__Sequence__fini(&msg->source_words);
  // request_id
  // worker_name
  rosidl_runtime_c__String__fini(&msg->worker_name);
  // start_timestamp_ns
  // completion_timestamp_ns
  // inference_duration_ms
  // session_id
  rosidl_runtime_c__String__fini(&msg->session_id);
}

bool
interfaces__msg__Inlg__are_equal(const interfaces__msg__Inlg * lhs, const interfaces__msg__Inlg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // reply
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reply), &(rhs->reply)))
  {
    return false;
  }
  // stage
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->stage), &(rhs->stage)))
  {
    return false;
  }
  // source_words
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->source_words), &(rhs->source_words)))
  {
    return false;
  }
  // request_id
  if (lhs->request_id != rhs->request_id) {
    return false;
  }
  // worker_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->worker_name), &(rhs->worker_name)))
  {
    return false;
  }
  // start_timestamp_ns
  if (lhs->start_timestamp_ns != rhs->start_timestamp_ns) {
    return false;
  }
  // completion_timestamp_ns
  if (lhs->completion_timestamp_ns != rhs->completion_timestamp_ns) {
    return false;
  }
  // inference_duration_ms
  if (lhs->inference_duration_ms != rhs->inference_duration_ms) {
    return false;
  }
  // session_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->session_id), &(rhs->session_id)))
  {
    return false;
  }
  return true;
}

bool
interfaces__msg__Inlg__copy(
  const interfaces__msg__Inlg * input,
  interfaces__msg__Inlg * output)
{
  if (!input || !output) {
    return false;
  }
  // reply
  if (!rosidl_runtime_c__String__copy(
      &(input->reply), &(output->reply)))
  {
    return false;
  }
  // stage
  if (!rosidl_runtime_c__String__copy(
      &(input->stage), &(output->stage)))
  {
    return false;
  }
  // source_words
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->source_words), &(output->source_words)))
  {
    return false;
  }
  // request_id
  output->request_id = input->request_id;
  // worker_name
  if (!rosidl_runtime_c__String__copy(
      &(input->worker_name), &(output->worker_name)))
  {
    return false;
  }
  // start_timestamp_ns
  output->start_timestamp_ns = input->start_timestamp_ns;
  // completion_timestamp_ns
  output->completion_timestamp_ns = input->completion_timestamp_ns;
  // inference_duration_ms
  output->inference_duration_ms = input->inference_duration_ms;
  // session_id
  if (!rosidl_runtime_c__String__copy(
      &(input->session_id), &(output->session_id)))
  {
    return false;
  }
  return true;
}

interfaces__msg__Inlg *
interfaces__msg__Inlg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__Inlg * msg = (interfaces__msg__Inlg *)allocator.allocate(sizeof(interfaces__msg__Inlg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__msg__Inlg));
  bool success = interfaces__msg__Inlg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interfaces__msg__Inlg__destroy(interfaces__msg__Inlg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interfaces__msg__Inlg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interfaces__msg__Inlg__Sequence__init(interfaces__msg__Inlg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__Inlg * data = NULL;

  if (size) {
    data = (interfaces__msg__Inlg *)allocator.zero_allocate(size, sizeof(interfaces__msg__Inlg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__msg__Inlg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__msg__Inlg__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
interfaces__msg__Inlg__Sequence__fini(interfaces__msg__Inlg__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interfaces__msg__Inlg__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

interfaces__msg__Inlg__Sequence *
interfaces__msg__Inlg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__Inlg__Sequence * array = (interfaces__msg__Inlg__Sequence *)allocator.allocate(sizeof(interfaces__msg__Inlg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interfaces__msg__Inlg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interfaces__msg__Inlg__Sequence__destroy(interfaces__msg__Inlg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interfaces__msg__Inlg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interfaces__msg__Inlg__Sequence__are_equal(const interfaces__msg__Inlg__Sequence * lhs, const interfaces__msg__Inlg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interfaces__msg__Inlg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interfaces__msg__Inlg__Sequence__copy(
  const interfaces__msg__Inlg__Sequence * input,
  interfaces__msg__Inlg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interfaces__msg__Inlg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interfaces__msg__Inlg * data =
      (interfaces__msg__Inlg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interfaces__msg__Inlg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interfaces__msg__Inlg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interfaces__msg__Inlg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
