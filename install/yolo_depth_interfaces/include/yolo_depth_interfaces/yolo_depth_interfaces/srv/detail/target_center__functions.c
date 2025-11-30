// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yolo_depth_interfaces:srv/TargetCenter.idl
// generated code does not contain a copyright notice
#include "yolo_depth_interfaces/srv/detail/target_center__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `target`
#include "rosidl_runtime_c/string_functions.h"

bool
yolo_depth_interfaces__srv__TargetCenter_Request__init(yolo_depth_interfaces__srv__TargetCenter_Request * msg)
{
  if (!msg) {
    return false;
  }
  // target
  if (!rosidl_runtime_c__String__init(&msg->target)) {
    yolo_depth_interfaces__srv__TargetCenter_Request__fini(msg);
    return false;
  }
  // center_x
  // center_y
  // distance_m
  return true;
}

void
yolo_depth_interfaces__srv__TargetCenter_Request__fini(yolo_depth_interfaces__srv__TargetCenter_Request * msg)
{
  if (!msg) {
    return;
  }
  // target
  rosidl_runtime_c__String__fini(&msg->target);
  // center_x
  // center_y
  // distance_m
}

bool
yolo_depth_interfaces__srv__TargetCenter_Request__are_equal(const yolo_depth_interfaces__srv__TargetCenter_Request * lhs, const yolo_depth_interfaces__srv__TargetCenter_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->target), &(rhs->target)))
  {
    return false;
  }
  // center_x
  if (lhs->center_x != rhs->center_x) {
    return false;
  }
  // center_y
  if (lhs->center_y != rhs->center_y) {
    return false;
  }
  // distance_m
  if (lhs->distance_m != rhs->distance_m) {
    return false;
  }
  return true;
}

bool
yolo_depth_interfaces__srv__TargetCenter_Request__copy(
  const yolo_depth_interfaces__srv__TargetCenter_Request * input,
  yolo_depth_interfaces__srv__TargetCenter_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // target
  if (!rosidl_runtime_c__String__copy(
      &(input->target), &(output->target)))
  {
    return false;
  }
  // center_x
  output->center_x = input->center_x;
  // center_y
  output->center_y = input->center_y;
  // distance_m
  output->distance_m = input->distance_m;
  return true;
}

yolo_depth_interfaces__srv__TargetCenter_Request *
yolo_depth_interfaces__srv__TargetCenter_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_depth_interfaces__srv__TargetCenter_Request * msg = (yolo_depth_interfaces__srv__TargetCenter_Request *)allocator.allocate(sizeof(yolo_depth_interfaces__srv__TargetCenter_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yolo_depth_interfaces__srv__TargetCenter_Request));
  bool success = yolo_depth_interfaces__srv__TargetCenter_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yolo_depth_interfaces__srv__TargetCenter_Request__destroy(yolo_depth_interfaces__srv__TargetCenter_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yolo_depth_interfaces__srv__TargetCenter_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yolo_depth_interfaces__srv__TargetCenter_Request__Sequence__init(yolo_depth_interfaces__srv__TargetCenter_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_depth_interfaces__srv__TargetCenter_Request * data = NULL;

  if (size) {
    data = (yolo_depth_interfaces__srv__TargetCenter_Request *)allocator.zero_allocate(size, sizeof(yolo_depth_interfaces__srv__TargetCenter_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yolo_depth_interfaces__srv__TargetCenter_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yolo_depth_interfaces__srv__TargetCenter_Request__fini(&data[i - 1]);
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
yolo_depth_interfaces__srv__TargetCenter_Request__Sequence__fini(yolo_depth_interfaces__srv__TargetCenter_Request__Sequence * array)
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
      yolo_depth_interfaces__srv__TargetCenter_Request__fini(&array->data[i]);
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

yolo_depth_interfaces__srv__TargetCenter_Request__Sequence *
yolo_depth_interfaces__srv__TargetCenter_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_depth_interfaces__srv__TargetCenter_Request__Sequence * array = (yolo_depth_interfaces__srv__TargetCenter_Request__Sequence *)allocator.allocate(sizeof(yolo_depth_interfaces__srv__TargetCenter_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yolo_depth_interfaces__srv__TargetCenter_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yolo_depth_interfaces__srv__TargetCenter_Request__Sequence__destroy(yolo_depth_interfaces__srv__TargetCenter_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yolo_depth_interfaces__srv__TargetCenter_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yolo_depth_interfaces__srv__TargetCenter_Request__Sequence__are_equal(const yolo_depth_interfaces__srv__TargetCenter_Request__Sequence * lhs, const yolo_depth_interfaces__srv__TargetCenter_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yolo_depth_interfaces__srv__TargetCenter_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yolo_depth_interfaces__srv__TargetCenter_Request__Sequence__copy(
  const yolo_depth_interfaces__srv__TargetCenter_Request__Sequence * input,
  yolo_depth_interfaces__srv__TargetCenter_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yolo_depth_interfaces__srv__TargetCenter_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yolo_depth_interfaces__srv__TargetCenter_Request * data =
      (yolo_depth_interfaces__srv__TargetCenter_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yolo_depth_interfaces__srv__TargetCenter_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yolo_depth_interfaces__srv__TargetCenter_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yolo_depth_interfaces__srv__TargetCenter_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
yolo_depth_interfaces__srv__TargetCenter_Response__init(yolo_depth_interfaces__srv__TargetCenter_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    yolo_depth_interfaces__srv__TargetCenter_Response__fini(msg);
    return false;
  }
  return true;
}

void
yolo_depth_interfaces__srv__TargetCenter_Response__fini(yolo_depth_interfaces__srv__TargetCenter_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
yolo_depth_interfaces__srv__TargetCenter_Response__are_equal(const yolo_depth_interfaces__srv__TargetCenter_Response * lhs, const yolo_depth_interfaces__srv__TargetCenter_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
yolo_depth_interfaces__srv__TargetCenter_Response__copy(
  const yolo_depth_interfaces__srv__TargetCenter_Response * input,
  yolo_depth_interfaces__srv__TargetCenter_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

yolo_depth_interfaces__srv__TargetCenter_Response *
yolo_depth_interfaces__srv__TargetCenter_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_depth_interfaces__srv__TargetCenter_Response * msg = (yolo_depth_interfaces__srv__TargetCenter_Response *)allocator.allocate(sizeof(yolo_depth_interfaces__srv__TargetCenter_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yolo_depth_interfaces__srv__TargetCenter_Response));
  bool success = yolo_depth_interfaces__srv__TargetCenter_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yolo_depth_interfaces__srv__TargetCenter_Response__destroy(yolo_depth_interfaces__srv__TargetCenter_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yolo_depth_interfaces__srv__TargetCenter_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yolo_depth_interfaces__srv__TargetCenter_Response__Sequence__init(yolo_depth_interfaces__srv__TargetCenter_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_depth_interfaces__srv__TargetCenter_Response * data = NULL;

  if (size) {
    data = (yolo_depth_interfaces__srv__TargetCenter_Response *)allocator.zero_allocate(size, sizeof(yolo_depth_interfaces__srv__TargetCenter_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yolo_depth_interfaces__srv__TargetCenter_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yolo_depth_interfaces__srv__TargetCenter_Response__fini(&data[i - 1]);
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
yolo_depth_interfaces__srv__TargetCenter_Response__Sequence__fini(yolo_depth_interfaces__srv__TargetCenter_Response__Sequence * array)
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
      yolo_depth_interfaces__srv__TargetCenter_Response__fini(&array->data[i]);
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

yolo_depth_interfaces__srv__TargetCenter_Response__Sequence *
yolo_depth_interfaces__srv__TargetCenter_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_depth_interfaces__srv__TargetCenter_Response__Sequence * array = (yolo_depth_interfaces__srv__TargetCenter_Response__Sequence *)allocator.allocate(sizeof(yolo_depth_interfaces__srv__TargetCenter_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yolo_depth_interfaces__srv__TargetCenter_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yolo_depth_interfaces__srv__TargetCenter_Response__Sequence__destroy(yolo_depth_interfaces__srv__TargetCenter_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yolo_depth_interfaces__srv__TargetCenter_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yolo_depth_interfaces__srv__TargetCenter_Response__Sequence__are_equal(const yolo_depth_interfaces__srv__TargetCenter_Response__Sequence * lhs, const yolo_depth_interfaces__srv__TargetCenter_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yolo_depth_interfaces__srv__TargetCenter_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yolo_depth_interfaces__srv__TargetCenter_Response__Sequence__copy(
  const yolo_depth_interfaces__srv__TargetCenter_Response__Sequence * input,
  yolo_depth_interfaces__srv__TargetCenter_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yolo_depth_interfaces__srv__TargetCenter_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yolo_depth_interfaces__srv__TargetCenter_Response * data =
      (yolo_depth_interfaces__srv__TargetCenter_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yolo_depth_interfaces__srv__TargetCenter_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yolo_depth_interfaces__srv__TargetCenter_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yolo_depth_interfaces__srv__TargetCenter_Response__copy(
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
// #include "yolo_depth_interfaces/srv/detail/target_center__functions.h"

bool
yolo_depth_interfaces__srv__TargetCenter_Event__init(yolo_depth_interfaces__srv__TargetCenter_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    yolo_depth_interfaces__srv__TargetCenter_Event__fini(msg);
    return false;
  }
  // request
  if (!yolo_depth_interfaces__srv__TargetCenter_Request__Sequence__init(&msg->request, 0)) {
    yolo_depth_interfaces__srv__TargetCenter_Event__fini(msg);
    return false;
  }
  // response
  if (!yolo_depth_interfaces__srv__TargetCenter_Response__Sequence__init(&msg->response, 0)) {
    yolo_depth_interfaces__srv__TargetCenter_Event__fini(msg);
    return false;
  }
  return true;
}

void
yolo_depth_interfaces__srv__TargetCenter_Event__fini(yolo_depth_interfaces__srv__TargetCenter_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  yolo_depth_interfaces__srv__TargetCenter_Request__Sequence__fini(&msg->request);
  // response
  yolo_depth_interfaces__srv__TargetCenter_Response__Sequence__fini(&msg->response);
}

bool
yolo_depth_interfaces__srv__TargetCenter_Event__are_equal(const yolo_depth_interfaces__srv__TargetCenter_Event * lhs, const yolo_depth_interfaces__srv__TargetCenter_Event * rhs)
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
  if (!yolo_depth_interfaces__srv__TargetCenter_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!yolo_depth_interfaces__srv__TargetCenter_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
yolo_depth_interfaces__srv__TargetCenter_Event__copy(
  const yolo_depth_interfaces__srv__TargetCenter_Event * input,
  yolo_depth_interfaces__srv__TargetCenter_Event * output)
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
  if (!yolo_depth_interfaces__srv__TargetCenter_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!yolo_depth_interfaces__srv__TargetCenter_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

yolo_depth_interfaces__srv__TargetCenter_Event *
yolo_depth_interfaces__srv__TargetCenter_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_depth_interfaces__srv__TargetCenter_Event * msg = (yolo_depth_interfaces__srv__TargetCenter_Event *)allocator.allocate(sizeof(yolo_depth_interfaces__srv__TargetCenter_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yolo_depth_interfaces__srv__TargetCenter_Event));
  bool success = yolo_depth_interfaces__srv__TargetCenter_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yolo_depth_interfaces__srv__TargetCenter_Event__destroy(yolo_depth_interfaces__srv__TargetCenter_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yolo_depth_interfaces__srv__TargetCenter_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yolo_depth_interfaces__srv__TargetCenter_Event__Sequence__init(yolo_depth_interfaces__srv__TargetCenter_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_depth_interfaces__srv__TargetCenter_Event * data = NULL;

  if (size) {
    data = (yolo_depth_interfaces__srv__TargetCenter_Event *)allocator.zero_allocate(size, sizeof(yolo_depth_interfaces__srv__TargetCenter_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yolo_depth_interfaces__srv__TargetCenter_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yolo_depth_interfaces__srv__TargetCenter_Event__fini(&data[i - 1]);
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
yolo_depth_interfaces__srv__TargetCenter_Event__Sequence__fini(yolo_depth_interfaces__srv__TargetCenter_Event__Sequence * array)
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
      yolo_depth_interfaces__srv__TargetCenter_Event__fini(&array->data[i]);
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

yolo_depth_interfaces__srv__TargetCenter_Event__Sequence *
yolo_depth_interfaces__srv__TargetCenter_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_depth_interfaces__srv__TargetCenter_Event__Sequence * array = (yolo_depth_interfaces__srv__TargetCenter_Event__Sequence *)allocator.allocate(sizeof(yolo_depth_interfaces__srv__TargetCenter_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yolo_depth_interfaces__srv__TargetCenter_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yolo_depth_interfaces__srv__TargetCenter_Event__Sequence__destroy(yolo_depth_interfaces__srv__TargetCenter_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yolo_depth_interfaces__srv__TargetCenter_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yolo_depth_interfaces__srv__TargetCenter_Event__Sequence__are_equal(const yolo_depth_interfaces__srv__TargetCenter_Event__Sequence * lhs, const yolo_depth_interfaces__srv__TargetCenter_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yolo_depth_interfaces__srv__TargetCenter_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yolo_depth_interfaces__srv__TargetCenter_Event__Sequence__copy(
  const yolo_depth_interfaces__srv__TargetCenter_Event__Sequence * input,
  yolo_depth_interfaces__srv__TargetCenter_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yolo_depth_interfaces__srv__TargetCenter_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yolo_depth_interfaces__srv__TargetCenter_Event * data =
      (yolo_depth_interfaces__srv__TargetCenter_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yolo_depth_interfaces__srv__TargetCenter_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yolo_depth_interfaces__srv__TargetCenter_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yolo_depth_interfaces__srv__TargetCenter_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
