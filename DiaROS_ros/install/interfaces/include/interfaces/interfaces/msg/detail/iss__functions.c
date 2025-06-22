// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces:msg/Iss.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/iss__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `timestamp`
#include "rosidl_runtime_c/string_functions.h"

bool
interfaces__msg__Iss__init(interfaces__msg__Iss * msg)
{
  if (!msg) {
    return false;
  }
  // is_speaking
  // timestamp
  if (!rosidl_runtime_c__String__init(&msg->timestamp)) {
    interfaces__msg__Iss__fini(msg);
    return false;
  }
  return true;
}

void
interfaces__msg__Iss__fini(interfaces__msg__Iss * msg)
{
  if (!msg) {
    return;
  }
  // is_speaking
  // timestamp
  rosidl_runtime_c__String__fini(&msg->timestamp);
}

bool
interfaces__msg__Iss__are_equal(const interfaces__msg__Iss * lhs, const interfaces__msg__Iss * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // is_speaking
  if (lhs->is_speaking != rhs->is_speaking) {
    return false;
  }
  // timestamp
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->timestamp), &(rhs->timestamp)))
  {
    return false;
  }
  return true;
}

bool
interfaces__msg__Iss__copy(
  const interfaces__msg__Iss * input,
  interfaces__msg__Iss * output)
{
  if (!input || !output) {
    return false;
  }
  // is_speaking
  output->is_speaking = input->is_speaking;
  // timestamp
  if (!rosidl_runtime_c__String__copy(
      &(input->timestamp), &(output->timestamp)))
  {
    return false;
  }
  return true;
}

interfaces__msg__Iss *
interfaces__msg__Iss__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__Iss * msg = (interfaces__msg__Iss *)allocator.allocate(sizeof(interfaces__msg__Iss), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__msg__Iss));
  bool success = interfaces__msg__Iss__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interfaces__msg__Iss__destroy(interfaces__msg__Iss * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interfaces__msg__Iss__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interfaces__msg__Iss__Sequence__init(interfaces__msg__Iss__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__Iss * data = NULL;

  if (size) {
    data = (interfaces__msg__Iss *)allocator.zero_allocate(size, sizeof(interfaces__msg__Iss), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__msg__Iss__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__msg__Iss__fini(&data[i - 1]);
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
interfaces__msg__Iss__Sequence__fini(interfaces__msg__Iss__Sequence * array)
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
      interfaces__msg__Iss__fini(&array->data[i]);
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

interfaces__msg__Iss__Sequence *
interfaces__msg__Iss__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__Iss__Sequence * array = (interfaces__msg__Iss__Sequence *)allocator.allocate(sizeof(interfaces__msg__Iss__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interfaces__msg__Iss__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interfaces__msg__Iss__Sequence__destroy(interfaces__msg__Iss__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interfaces__msg__Iss__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interfaces__msg__Iss__Sequence__are_equal(const interfaces__msg__Iss__Sequence * lhs, const interfaces__msg__Iss__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interfaces__msg__Iss__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interfaces__msg__Iss__Sequence__copy(
  const interfaces__msg__Iss__Sequence * input,
  interfaces__msg__Iss__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interfaces__msg__Iss);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interfaces__msg__Iss * data =
      (interfaces__msg__Iss *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interfaces__msg__Iss__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interfaces__msg__Iss__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interfaces__msg__Iss__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
