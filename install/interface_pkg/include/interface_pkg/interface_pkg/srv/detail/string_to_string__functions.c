// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interface_pkg:srv/StringToString.idl
// generated code does not contain a copyright notice
#include "interface_pkg/srv/detail/string_to_string__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `input`
#include "rosidl_runtime_c/string_functions.h"

bool
interface_pkg__srv__StringToString_Request__init(interface_pkg__srv__StringToString_Request * msg)
{
  if (!msg) {
    return false;
  }
  // input
  if (!rosidl_runtime_c__String__init(&msg->input)) {
    interface_pkg__srv__StringToString_Request__fini(msg);
    return false;
  }
  return true;
}

void
interface_pkg__srv__StringToString_Request__fini(interface_pkg__srv__StringToString_Request * msg)
{
  if (!msg) {
    return;
  }
  // input
  rosidl_runtime_c__String__fini(&msg->input);
}

bool
interface_pkg__srv__StringToString_Request__are_equal(const interface_pkg__srv__StringToString_Request * lhs, const interface_pkg__srv__StringToString_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // input
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->input), &(rhs->input)))
  {
    return false;
  }
  return true;
}

bool
interface_pkg__srv__StringToString_Request__copy(
  const interface_pkg__srv__StringToString_Request * input,
  interface_pkg__srv__StringToString_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // input
  if (!rosidl_runtime_c__String__copy(
      &(input->input), &(output->input)))
  {
    return false;
  }
  return true;
}

interface_pkg__srv__StringToString_Request *
interface_pkg__srv__StringToString_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__StringToString_Request * msg = (interface_pkg__srv__StringToString_Request *)allocator.allocate(sizeof(interface_pkg__srv__StringToString_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface_pkg__srv__StringToString_Request));
  bool success = interface_pkg__srv__StringToString_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface_pkg__srv__StringToString_Request__destroy(interface_pkg__srv__StringToString_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface_pkg__srv__StringToString_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface_pkg__srv__StringToString_Request__Sequence__init(interface_pkg__srv__StringToString_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__StringToString_Request * data = NULL;

  if (size) {
    data = (interface_pkg__srv__StringToString_Request *)allocator.zero_allocate(size, sizeof(interface_pkg__srv__StringToString_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface_pkg__srv__StringToString_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface_pkg__srv__StringToString_Request__fini(&data[i - 1]);
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
interface_pkg__srv__StringToString_Request__Sequence__fini(interface_pkg__srv__StringToString_Request__Sequence * array)
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
      interface_pkg__srv__StringToString_Request__fini(&array->data[i]);
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

interface_pkg__srv__StringToString_Request__Sequence *
interface_pkg__srv__StringToString_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__StringToString_Request__Sequence * array = (interface_pkg__srv__StringToString_Request__Sequence *)allocator.allocate(sizeof(interface_pkg__srv__StringToString_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface_pkg__srv__StringToString_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface_pkg__srv__StringToString_Request__Sequence__destroy(interface_pkg__srv__StringToString_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface_pkg__srv__StringToString_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface_pkg__srv__StringToString_Request__Sequence__are_equal(const interface_pkg__srv__StringToString_Request__Sequence * lhs, const interface_pkg__srv__StringToString_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface_pkg__srv__StringToString_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface_pkg__srv__StringToString_Request__Sequence__copy(
  const interface_pkg__srv__StringToString_Request__Sequence * input,
  interface_pkg__srv__StringToString_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface_pkg__srv__StringToString_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interface_pkg__srv__StringToString_Request * data =
      (interface_pkg__srv__StringToString_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface_pkg__srv__StringToString_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interface_pkg__srv__StringToString_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface_pkg__srv__StringToString_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `output`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
interface_pkg__srv__StringToString_Response__init(interface_pkg__srv__StringToString_Response * msg)
{
  if (!msg) {
    return false;
  }
  // output
  if (!rosidl_runtime_c__String__init(&msg->output)) {
    interface_pkg__srv__StringToString_Response__fini(msg);
    return false;
  }
  return true;
}

void
interface_pkg__srv__StringToString_Response__fini(interface_pkg__srv__StringToString_Response * msg)
{
  if (!msg) {
    return;
  }
  // output
  rosidl_runtime_c__String__fini(&msg->output);
}

bool
interface_pkg__srv__StringToString_Response__are_equal(const interface_pkg__srv__StringToString_Response * lhs, const interface_pkg__srv__StringToString_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // output
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->output), &(rhs->output)))
  {
    return false;
  }
  return true;
}

bool
interface_pkg__srv__StringToString_Response__copy(
  const interface_pkg__srv__StringToString_Response * input,
  interface_pkg__srv__StringToString_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // output
  if (!rosidl_runtime_c__String__copy(
      &(input->output), &(output->output)))
  {
    return false;
  }
  return true;
}

interface_pkg__srv__StringToString_Response *
interface_pkg__srv__StringToString_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__StringToString_Response * msg = (interface_pkg__srv__StringToString_Response *)allocator.allocate(sizeof(interface_pkg__srv__StringToString_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface_pkg__srv__StringToString_Response));
  bool success = interface_pkg__srv__StringToString_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface_pkg__srv__StringToString_Response__destroy(interface_pkg__srv__StringToString_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface_pkg__srv__StringToString_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface_pkg__srv__StringToString_Response__Sequence__init(interface_pkg__srv__StringToString_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__StringToString_Response * data = NULL;

  if (size) {
    data = (interface_pkg__srv__StringToString_Response *)allocator.zero_allocate(size, sizeof(interface_pkg__srv__StringToString_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface_pkg__srv__StringToString_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface_pkg__srv__StringToString_Response__fini(&data[i - 1]);
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
interface_pkg__srv__StringToString_Response__Sequence__fini(interface_pkg__srv__StringToString_Response__Sequence * array)
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
      interface_pkg__srv__StringToString_Response__fini(&array->data[i]);
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

interface_pkg__srv__StringToString_Response__Sequence *
interface_pkg__srv__StringToString_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__StringToString_Response__Sequence * array = (interface_pkg__srv__StringToString_Response__Sequence *)allocator.allocate(sizeof(interface_pkg__srv__StringToString_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface_pkg__srv__StringToString_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface_pkg__srv__StringToString_Response__Sequence__destroy(interface_pkg__srv__StringToString_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface_pkg__srv__StringToString_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface_pkg__srv__StringToString_Response__Sequence__are_equal(const interface_pkg__srv__StringToString_Response__Sequence * lhs, const interface_pkg__srv__StringToString_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface_pkg__srv__StringToString_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface_pkg__srv__StringToString_Response__Sequence__copy(
  const interface_pkg__srv__StringToString_Response__Sequence * input,
  interface_pkg__srv__StringToString_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface_pkg__srv__StringToString_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interface_pkg__srv__StringToString_Response * data =
      (interface_pkg__srv__StringToString_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface_pkg__srv__StringToString_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interface_pkg__srv__StringToString_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface_pkg__srv__StringToString_Response__copy(
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
// #include "interface_pkg/srv/detail/string_to_string__functions.h"

bool
interface_pkg__srv__StringToString_Event__init(interface_pkg__srv__StringToString_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    interface_pkg__srv__StringToString_Event__fini(msg);
    return false;
  }
  // request
  if (!interface_pkg__srv__StringToString_Request__Sequence__init(&msg->request, 0)) {
    interface_pkg__srv__StringToString_Event__fini(msg);
    return false;
  }
  // response
  if (!interface_pkg__srv__StringToString_Response__Sequence__init(&msg->response, 0)) {
    interface_pkg__srv__StringToString_Event__fini(msg);
    return false;
  }
  return true;
}

void
interface_pkg__srv__StringToString_Event__fini(interface_pkg__srv__StringToString_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  interface_pkg__srv__StringToString_Request__Sequence__fini(&msg->request);
  // response
  interface_pkg__srv__StringToString_Response__Sequence__fini(&msg->response);
}

bool
interface_pkg__srv__StringToString_Event__are_equal(const interface_pkg__srv__StringToString_Event * lhs, const interface_pkg__srv__StringToString_Event * rhs)
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
  if (!interface_pkg__srv__StringToString_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!interface_pkg__srv__StringToString_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
interface_pkg__srv__StringToString_Event__copy(
  const interface_pkg__srv__StringToString_Event * input,
  interface_pkg__srv__StringToString_Event * output)
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
  if (!interface_pkg__srv__StringToString_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!interface_pkg__srv__StringToString_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

interface_pkg__srv__StringToString_Event *
interface_pkg__srv__StringToString_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__StringToString_Event * msg = (interface_pkg__srv__StringToString_Event *)allocator.allocate(sizeof(interface_pkg__srv__StringToString_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface_pkg__srv__StringToString_Event));
  bool success = interface_pkg__srv__StringToString_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface_pkg__srv__StringToString_Event__destroy(interface_pkg__srv__StringToString_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface_pkg__srv__StringToString_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface_pkg__srv__StringToString_Event__Sequence__init(interface_pkg__srv__StringToString_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__StringToString_Event * data = NULL;

  if (size) {
    data = (interface_pkg__srv__StringToString_Event *)allocator.zero_allocate(size, sizeof(interface_pkg__srv__StringToString_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface_pkg__srv__StringToString_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface_pkg__srv__StringToString_Event__fini(&data[i - 1]);
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
interface_pkg__srv__StringToString_Event__Sequence__fini(interface_pkg__srv__StringToString_Event__Sequence * array)
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
      interface_pkg__srv__StringToString_Event__fini(&array->data[i]);
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

interface_pkg__srv__StringToString_Event__Sequence *
interface_pkg__srv__StringToString_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_pkg__srv__StringToString_Event__Sequence * array = (interface_pkg__srv__StringToString_Event__Sequence *)allocator.allocate(sizeof(interface_pkg__srv__StringToString_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface_pkg__srv__StringToString_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface_pkg__srv__StringToString_Event__Sequence__destroy(interface_pkg__srv__StringToString_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface_pkg__srv__StringToString_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface_pkg__srv__StringToString_Event__Sequence__are_equal(const interface_pkg__srv__StringToString_Event__Sequence * lhs, const interface_pkg__srv__StringToString_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface_pkg__srv__StringToString_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface_pkg__srv__StringToString_Event__Sequence__copy(
  const interface_pkg__srv__StringToString_Event__Sequence * input,
  interface_pkg__srv__StringToString_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface_pkg__srv__StringToString_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interface_pkg__srv__StringToString_Event * data =
      (interface_pkg__srv__StringToString_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface_pkg__srv__StringToString_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interface_pkg__srv__StringToString_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface_pkg__srv__StringToString_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
