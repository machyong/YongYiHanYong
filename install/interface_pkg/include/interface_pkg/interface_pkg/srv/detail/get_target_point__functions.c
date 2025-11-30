// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interface_pkg:srv/GetTargetPoint.idl
// generated code does not contain a copyright notice
#include "interface_pkg/srv/detail/get_target_point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
interface_pkg__srv__GetTargetPoint_Request__init(interface_pkg__srv__GetTargetPoint_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
interface_pkg__srv__GetTargetPoint_Request__fini(interface_pkg__srv__GetTargetPoint_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
interface_pkg__srv__GetTargetPoint_Request__are_equal(const interface_pkg__srv__GetTargetPoint_Request * lhs, const interface_pkg__srv__GetTargetPoint_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
interface_pkg__srv__GetTargetPoint_Request__copy(
  const interface_pkg__srv__GetTargetPoint_Request * input,
  interface_pkg__srv__GetTargetPoint_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

interface_pkg__srv__GetTargetPoint_Request *
interface_pkg__srv__GetTargetPoint_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__GetTargetPoint_Request * msg = (interface_pkg__srv__GetTargetPoint_Request *)allocator.allocate(sizeof(interface_pkg__srv__GetTargetPoint_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface_pkg__srv__GetTargetPoint_Request));
  bool success = interface_pkg__srv__GetTargetPoint_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface_pkg__srv__GetTargetPoint_Request__destroy(interface_pkg__srv__GetTargetPoint_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface_pkg__srv__GetTargetPoint_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface_pkg__srv__GetTargetPoint_Request__Sequence__init(interface_pkg__srv__GetTargetPoint_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__GetTargetPoint_Request * data = NULL;

  if (size) {
    data = (interface_pkg__srv__GetTargetPoint_Request *)allocator.zero_allocate(size, sizeof(interface_pkg__srv__GetTargetPoint_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface_pkg__srv__GetTargetPoint_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface_pkg__srv__GetTargetPoint_Request__fini(&data[i - 1]);
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
interface_pkg__srv__GetTargetPoint_Request__Sequence__fini(interface_pkg__srv__GetTargetPoint_Request__Sequence * array)
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
      interface_pkg__srv__GetTargetPoint_Request__fini(&array->data[i]);
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

interface_pkg__srv__GetTargetPoint_Request__Sequence *
interface_pkg__srv__GetTargetPoint_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__GetTargetPoint_Request__Sequence * array = (interface_pkg__srv__GetTargetPoint_Request__Sequence *)allocator.allocate(sizeof(interface_pkg__srv__GetTargetPoint_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface_pkg__srv__GetTargetPoint_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface_pkg__srv__GetTargetPoint_Request__Sequence__destroy(interface_pkg__srv__GetTargetPoint_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface_pkg__srv__GetTargetPoint_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface_pkg__srv__GetTargetPoint_Request__Sequence__are_equal(const interface_pkg__srv__GetTargetPoint_Request__Sequence * lhs, const interface_pkg__srv__GetTargetPoint_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface_pkg__srv__GetTargetPoint_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface_pkg__srv__GetTargetPoint_Request__Sequence__copy(
  const interface_pkg__srv__GetTargetPoint_Request__Sequence * input,
  interface_pkg__srv__GetTargetPoint_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface_pkg__srv__GetTargetPoint_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interface_pkg__srv__GetTargetPoint_Request * data =
      (interface_pkg__srv__GetTargetPoint_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface_pkg__srv__GetTargetPoint_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interface_pkg__srv__GetTargetPoint_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface_pkg__srv__GetTargetPoint_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `target_point`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `class_name`
#include "rosidl_runtime_c/string_functions.h"

bool
interface_pkg__srv__GetTargetPoint_Response__init(interface_pkg__srv__GetTargetPoint_Response * msg)
{
  if (!msg) {
    return false;
  }
  // target_point
  if (!geometry_msgs__msg__Point__init(&msg->target_point)) {
    interface_pkg__srv__GetTargetPoint_Response__fini(msg);
    return false;
  }
  // class_name
  if (!rosidl_runtime_c__String__init(&msg->class_name)) {
    interface_pkg__srv__GetTargetPoint_Response__fini(msg);
    return false;
  }
  // confidence
  return true;
}

void
interface_pkg__srv__GetTargetPoint_Response__fini(interface_pkg__srv__GetTargetPoint_Response * msg)
{
  if (!msg) {
    return;
  }
  // target_point
  geometry_msgs__msg__Point__fini(&msg->target_point);
  // class_name
  rosidl_runtime_c__String__fini(&msg->class_name);
  // confidence
}

bool
interface_pkg__srv__GetTargetPoint_Response__are_equal(const interface_pkg__srv__GetTargetPoint_Response * lhs, const interface_pkg__srv__GetTargetPoint_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target_point
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->target_point), &(rhs->target_point)))
  {
    return false;
  }
  // class_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->class_name), &(rhs->class_name)))
  {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  return true;
}

bool
interface_pkg__srv__GetTargetPoint_Response__copy(
  const interface_pkg__srv__GetTargetPoint_Response * input,
  interface_pkg__srv__GetTargetPoint_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // target_point
  if (!geometry_msgs__msg__Point__copy(
      &(input->target_point), &(output->target_point)))
  {
    return false;
  }
  // class_name
  if (!rosidl_runtime_c__String__copy(
      &(input->class_name), &(output->class_name)))
  {
    return false;
  }
  // confidence
  output->confidence = input->confidence;
  return true;
}

interface_pkg__srv__GetTargetPoint_Response *
interface_pkg__srv__GetTargetPoint_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__GetTargetPoint_Response * msg = (interface_pkg__srv__GetTargetPoint_Response *)allocator.allocate(sizeof(interface_pkg__srv__GetTargetPoint_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface_pkg__srv__GetTargetPoint_Response));
  bool success = interface_pkg__srv__GetTargetPoint_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface_pkg__srv__GetTargetPoint_Response__destroy(interface_pkg__srv__GetTargetPoint_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface_pkg__srv__GetTargetPoint_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface_pkg__srv__GetTargetPoint_Response__Sequence__init(interface_pkg__srv__GetTargetPoint_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__GetTargetPoint_Response * data = NULL;

  if (size) {
    data = (interface_pkg__srv__GetTargetPoint_Response *)allocator.zero_allocate(size, sizeof(interface_pkg__srv__GetTargetPoint_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface_pkg__srv__GetTargetPoint_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface_pkg__srv__GetTargetPoint_Response__fini(&data[i - 1]);
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
interface_pkg__srv__GetTargetPoint_Response__Sequence__fini(interface_pkg__srv__GetTargetPoint_Response__Sequence * array)
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
      interface_pkg__srv__GetTargetPoint_Response__fini(&array->data[i]);
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

interface_pkg__srv__GetTargetPoint_Response__Sequence *
interface_pkg__srv__GetTargetPoint_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__GetTargetPoint_Response__Sequence * array = (interface_pkg__srv__GetTargetPoint_Response__Sequence *)allocator.allocate(sizeof(interface_pkg__srv__GetTargetPoint_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface_pkg__srv__GetTargetPoint_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface_pkg__srv__GetTargetPoint_Response__Sequence__destroy(interface_pkg__srv__GetTargetPoint_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface_pkg__srv__GetTargetPoint_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface_pkg__srv__GetTargetPoint_Response__Sequence__are_equal(const interface_pkg__srv__GetTargetPoint_Response__Sequence * lhs, const interface_pkg__srv__GetTargetPoint_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface_pkg__srv__GetTargetPoint_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface_pkg__srv__GetTargetPoint_Response__Sequence__copy(
  const interface_pkg__srv__GetTargetPoint_Response__Sequence * input,
  interface_pkg__srv__GetTargetPoint_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface_pkg__srv__GetTargetPoint_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interface_pkg__srv__GetTargetPoint_Response * data =
      (interface_pkg__srv__GetTargetPoint_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface_pkg__srv__GetTargetPoint_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interface_pkg__srv__GetTargetPoint_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface_pkg__srv__GetTargetPoint_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "interface_pkg/srv/detail/get_target_point__functions.h"

bool
interface_pkg__srv__GetTargetPoint_Event__init(interface_pkg__srv__GetTargetPoint_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    interface_pkg__srv__GetTargetPoint_Event__fini(msg);
    return false;
  }
  // request
  if (!interface_pkg__srv__GetTargetPoint_Request__Sequence__init(&msg->request, 0)) {
    interface_pkg__srv__GetTargetPoint_Event__fini(msg);
    return false;
  }
  // response
  if (!interface_pkg__srv__GetTargetPoint_Response__Sequence__init(&msg->response, 0)) {
    interface_pkg__srv__GetTargetPoint_Event__fini(msg);
    return false;
  }
  return true;
}

void
interface_pkg__srv__GetTargetPoint_Event__fini(interface_pkg__srv__GetTargetPoint_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  interface_pkg__srv__GetTargetPoint_Request__Sequence__fini(&msg->request);
  // response
  interface_pkg__srv__GetTargetPoint_Response__Sequence__fini(&msg->response);
}

bool
interface_pkg__srv__GetTargetPoint_Event__are_equal(const interface_pkg__srv__GetTargetPoint_Event * lhs, const interface_pkg__srv__GetTargetPoint_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!interface_pkg__srv__GetTargetPoint_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!interface_pkg__srv__GetTargetPoint_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
interface_pkg__srv__GetTargetPoint_Event__copy(
  const interface_pkg__srv__GetTargetPoint_Event * input,
  interface_pkg__srv__GetTargetPoint_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!interface_pkg__srv__GetTargetPoint_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!interface_pkg__srv__GetTargetPoint_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

interface_pkg__srv__GetTargetPoint_Event *
interface_pkg__srv__GetTargetPoint_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__GetTargetPoint_Event * msg = (interface_pkg__srv__GetTargetPoint_Event *)allocator.allocate(sizeof(interface_pkg__srv__GetTargetPoint_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface_pkg__srv__GetTargetPoint_Event));
  bool success = interface_pkg__srv__GetTargetPoint_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface_pkg__srv__GetTargetPoint_Event__destroy(interface_pkg__srv__GetTargetPoint_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface_pkg__srv__GetTargetPoint_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface_pkg__srv__GetTargetPoint_Event__Sequence__init(interface_pkg__srv__GetTargetPoint_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__GetTargetPoint_Event * data = NULL;

  if (size) {
    data = (interface_pkg__srv__GetTargetPoint_Event *)allocator.zero_allocate(size, sizeof(interface_pkg__srv__GetTargetPoint_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface_pkg__srv__GetTargetPoint_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface_pkg__srv__GetTargetPoint_Event__fini(&data[i - 1]);
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
interface_pkg__srv__GetTargetPoint_Event__Sequence__fini(interface_pkg__srv__GetTargetPoint_Event__Sequence * array)
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
      interface_pkg__srv__GetTargetPoint_Event__fini(&array->data[i]);
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

interface_pkg__srv__GetTargetPoint_Event__Sequence *
interface_pkg__srv__GetTargetPoint_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__GetTargetPoint_Event__Sequence * array = (interface_pkg__srv__GetTargetPoint_Event__Sequence *)allocator.allocate(sizeof(interface_pkg__srv__GetTargetPoint_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface_pkg__srv__GetTargetPoint_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface_pkg__srv__GetTargetPoint_Event__Sequence__destroy(interface_pkg__srv__GetTargetPoint_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface_pkg__srv__GetTargetPoint_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface_pkg__srv__GetTargetPoint_Event__Sequence__are_equal(const interface_pkg__srv__GetTargetPoint_Event__Sequence * lhs, const interface_pkg__srv__GetTargetPoint_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface_pkg__srv__GetTargetPoint_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface_pkg__srv__GetTargetPoint_Event__Sequence__copy(
  const interface_pkg__srv__GetTargetPoint_Event__Sequence * input,
  interface_pkg__srv__GetTargetPoint_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface_pkg__srv__GetTargetPoint_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interface_pkg__srv__GetTargetPoint_Event * data =
      (interface_pkg__srv__GetTargetPoint_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface_pkg__srv__GetTargetPoint_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interface_pkg__srv__GetTargetPoint_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface_pkg__srv__GetTargetPoint_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
