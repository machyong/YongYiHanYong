// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yolo_depth_interfaces:msg/ObjectDepthArray.idl
// generated code does not contain a copyright notice
#include "yolo_depth_interfaces/msg/detail/object_depth_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `objects`
#include "yolo_depth_interfaces/msg/detail/object_depth__functions.h"

bool
yolo_depth_interfaces__msg__ObjectDepthArray__init(yolo_depth_interfaces__msg__ObjectDepthArray * msg)
{
  if (!msg) {
    return false;
  }
  // objects
  if (!yolo_depth_interfaces__msg__ObjectDepth__Sequence__init(&msg->objects, 0)) {
    yolo_depth_interfaces__msg__ObjectDepthArray__fini(msg);
    return false;
  }
  return true;
}

void
yolo_depth_interfaces__msg__ObjectDepthArray__fini(yolo_depth_interfaces__msg__ObjectDepthArray * msg)
{
  if (!msg) {
    return;
  }
  // objects
  yolo_depth_interfaces__msg__ObjectDepth__Sequence__fini(&msg->objects);
}

bool
yolo_depth_interfaces__msg__ObjectDepthArray__are_equal(const yolo_depth_interfaces__msg__ObjectDepthArray * lhs, const yolo_depth_interfaces__msg__ObjectDepthArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // objects
  if (!yolo_depth_interfaces__msg__ObjectDepth__Sequence__are_equal(
      &(lhs->objects), &(rhs->objects)))
  {
    return false;
  }
  return true;
}

bool
yolo_depth_interfaces__msg__ObjectDepthArray__copy(
  const yolo_depth_interfaces__msg__ObjectDepthArray * input,
  yolo_depth_interfaces__msg__ObjectDepthArray * output)
{
  if (!input || !output) {
    return false;
  }
  // objects
  if (!yolo_depth_interfaces__msg__ObjectDepth__Sequence__copy(
      &(input->objects), &(output->objects)))
  {
    return false;
  }
  return true;
}

yolo_depth_interfaces__msg__ObjectDepthArray *
yolo_depth_interfaces__msg__ObjectDepthArray__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_depth_interfaces__msg__ObjectDepthArray * msg = (yolo_depth_interfaces__msg__ObjectDepthArray *)allocator.allocate(sizeof(yolo_depth_interfaces__msg__ObjectDepthArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yolo_depth_interfaces__msg__ObjectDepthArray));
  bool success = yolo_depth_interfaces__msg__ObjectDepthArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yolo_depth_interfaces__msg__ObjectDepthArray__destroy(yolo_depth_interfaces__msg__ObjectDepthArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yolo_depth_interfaces__msg__ObjectDepthArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yolo_depth_interfaces__msg__ObjectDepthArray__Sequence__init(yolo_depth_interfaces__msg__ObjectDepthArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_depth_interfaces__msg__ObjectDepthArray * data = NULL;

  if (size) {
    data = (yolo_depth_interfaces__msg__ObjectDepthArray *)allocator.zero_allocate(size, sizeof(yolo_depth_interfaces__msg__ObjectDepthArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yolo_depth_interfaces__msg__ObjectDepthArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yolo_depth_interfaces__msg__ObjectDepthArray__fini(&data[i - 1]);
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
yolo_depth_interfaces__msg__ObjectDepthArray__Sequence__fini(yolo_depth_interfaces__msg__ObjectDepthArray__Sequence * array)
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
      yolo_depth_interfaces__msg__ObjectDepthArray__fini(&array->data[i]);
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

yolo_depth_interfaces__msg__ObjectDepthArray__Sequence *
yolo_depth_interfaces__msg__ObjectDepthArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_depth_interfaces__msg__ObjectDepthArray__Sequence * array = (yolo_depth_interfaces__msg__ObjectDepthArray__Sequence *)allocator.allocate(sizeof(yolo_depth_interfaces__msg__ObjectDepthArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yolo_depth_interfaces__msg__ObjectDepthArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yolo_depth_interfaces__msg__ObjectDepthArray__Sequence__destroy(yolo_depth_interfaces__msg__ObjectDepthArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yolo_depth_interfaces__msg__ObjectDepthArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yolo_depth_interfaces__msg__ObjectDepthArray__Sequence__are_equal(const yolo_depth_interfaces__msg__ObjectDepthArray__Sequence * lhs, const yolo_depth_interfaces__msg__ObjectDepthArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yolo_depth_interfaces__msg__ObjectDepthArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yolo_depth_interfaces__msg__ObjectDepthArray__Sequence__copy(
  const yolo_depth_interfaces__msg__ObjectDepthArray__Sequence * input,
  yolo_depth_interfaces__msg__ObjectDepthArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yolo_depth_interfaces__msg__ObjectDepthArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yolo_depth_interfaces__msg__ObjectDepthArray * data =
      (yolo_depth_interfaces__msg__ObjectDepthArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yolo_depth_interfaces__msg__ObjectDepthArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yolo_depth_interfaces__msg__ObjectDepthArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yolo_depth_interfaces__msg__ObjectDepthArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
