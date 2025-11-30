// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yolo_depth_interfaces:msg/ObjectDepth.idl
// generated code does not contain a copyright notice
#include "yolo_depth_interfaces/msg/detail/object_depth__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `class_name`
#include "rosidl_runtime_c/string_functions.h"

bool
yolo_depth_interfaces__msg__ObjectDepth__init(yolo_depth_interfaces__msg__ObjectDepth * msg)
{
  if (!msg) {
    return false;
  }
  // class_name
  if (!rosidl_runtime_c__String__init(&msg->class_name)) {
    yolo_depth_interfaces__msg__ObjectDepth__fini(msg);
    return false;
  }
  // confidence
  // distance_m
  // x1
  // y1
  // x2
  // y2
  return true;
}

void
yolo_depth_interfaces__msg__ObjectDepth__fini(yolo_depth_interfaces__msg__ObjectDepth * msg)
{
  if (!msg) {
    return;
  }
  // class_name
  rosidl_runtime_c__String__fini(&msg->class_name);
  // confidence
  // distance_m
  // x1
  // y1
  // x2
  // y2
}

bool
yolo_depth_interfaces__msg__ObjectDepth__are_equal(const yolo_depth_interfaces__msg__ObjectDepth * lhs, const yolo_depth_interfaces__msg__ObjectDepth * rhs)
{
  if (!lhs || !rhs) {
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
  // distance_m
  if (lhs->distance_m != rhs->distance_m) {
    return false;
  }
  // x1
  if (lhs->x1 != rhs->x1) {
    return false;
  }
  // y1
  if (lhs->y1 != rhs->y1) {
    return false;
  }
  // x2
  if (lhs->x2 != rhs->x2) {
    return false;
  }
  // y2
  if (lhs->y2 != rhs->y2) {
    return false;
  }
  return true;
}

bool
yolo_depth_interfaces__msg__ObjectDepth__copy(
  const yolo_depth_interfaces__msg__ObjectDepth * input,
  yolo_depth_interfaces__msg__ObjectDepth * output)
{
  if (!input || !output) {
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
  // distance_m
  output->distance_m = input->distance_m;
  // x1
  output->x1 = input->x1;
  // y1
  output->y1 = input->y1;
  // x2
  output->x2 = input->x2;
  // y2
  output->y2 = input->y2;
  return true;
}

yolo_depth_interfaces__msg__ObjectDepth *
yolo_depth_interfaces__msg__ObjectDepth__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_depth_interfaces__msg__ObjectDepth * msg = (yolo_depth_interfaces__msg__ObjectDepth *)allocator.allocate(sizeof(yolo_depth_interfaces__msg__ObjectDepth), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yolo_depth_interfaces__msg__ObjectDepth));
  bool success = yolo_depth_interfaces__msg__ObjectDepth__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yolo_depth_interfaces__msg__ObjectDepth__destroy(yolo_depth_interfaces__msg__ObjectDepth * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yolo_depth_interfaces__msg__ObjectDepth__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yolo_depth_interfaces__msg__ObjectDepth__Sequence__init(yolo_depth_interfaces__msg__ObjectDepth__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_depth_interfaces__msg__ObjectDepth * data = NULL;

  if (size) {
    data = (yolo_depth_interfaces__msg__ObjectDepth *)allocator.zero_allocate(size, sizeof(yolo_depth_interfaces__msg__ObjectDepth), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yolo_depth_interfaces__msg__ObjectDepth__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yolo_depth_interfaces__msg__ObjectDepth__fini(&data[i - 1]);
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
yolo_depth_interfaces__msg__ObjectDepth__Sequence__fini(yolo_depth_interfaces__msg__ObjectDepth__Sequence * array)
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
      yolo_depth_interfaces__msg__ObjectDepth__fini(&array->data[i]);
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

yolo_depth_interfaces__msg__ObjectDepth__Sequence *
yolo_depth_interfaces__msg__ObjectDepth__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_depth_interfaces__msg__ObjectDepth__Sequence * array = (yolo_depth_interfaces__msg__ObjectDepth__Sequence *)allocator.allocate(sizeof(yolo_depth_interfaces__msg__ObjectDepth__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yolo_depth_interfaces__msg__ObjectDepth__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yolo_depth_interfaces__msg__ObjectDepth__Sequence__destroy(yolo_depth_interfaces__msg__ObjectDepth__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yolo_depth_interfaces__msg__ObjectDepth__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yolo_depth_interfaces__msg__ObjectDepth__Sequence__are_equal(const yolo_depth_interfaces__msg__ObjectDepth__Sequence * lhs, const yolo_depth_interfaces__msg__ObjectDepth__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yolo_depth_interfaces__msg__ObjectDepth__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yolo_depth_interfaces__msg__ObjectDepth__Sequence__copy(
  const yolo_depth_interfaces__msg__ObjectDepth__Sequence * input,
  yolo_depth_interfaces__msg__ObjectDepth__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yolo_depth_interfaces__msg__ObjectDepth);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yolo_depth_interfaces__msg__ObjectDepth * data =
      (yolo_depth_interfaces__msg__ObjectDepth *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yolo_depth_interfaces__msg__ObjectDepth__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yolo_depth_interfaces__msg__ObjectDepth__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yolo_depth_interfaces__msg__ObjectDepth__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
