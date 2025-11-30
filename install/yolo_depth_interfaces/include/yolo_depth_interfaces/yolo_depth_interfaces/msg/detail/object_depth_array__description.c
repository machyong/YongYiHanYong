// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from yolo_depth_interfaces:msg/ObjectDepthArray.idl
// generated code does not contain a copyright notice

#include "yolo_depth_interfaces/msg/detail/object_depth_array__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_yolo_depth_interfaces
const rosidl_type_hash_t *
yolo_depth_interfaces__msg__ObjectDepthArray__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x39, 0xf9, 0x0b, 0xfb, 0x59, 0xa0, 0x94, 0x1e,
      0xda, 0x07, 0xda, 0x58, 0xa2, 0xda, 0xca, 0x87,
      0x27, 0x30, 0xe8, 0x6b, 0xf7, 0xca, 0xab, 0x1c,
      0xca, 0x3d, 0x96, 0x6e, 0xad, 0xcd, 0xa8, 0xc1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "yolo_depth_interfaces/msg/detail/object_depth__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t yolo_depth_interfaces__msg__ObjectDepth__EXPECTED_HASH = {1, {
    0xb2, 0xd6, 0x43, 0x9b, 0x03, 0x9c, 0x0f, 0xe4,
    0xca, 0xa4, 0x36, 0x8c, 0xb1, 0x4d, 0x2a, 0x56,
    0x2f, 0x56, 0x35, 0x46, 0x3f, 0x0b, 0x08, 0x40,
    0xd9, 0x37, 0x6e, 0x64, 0x59, 0x0f, 0xd0, 0x8b,
  }};
#endif

static char yolo_depth_interfaces__msg__ObjectDepthArray__TYPE_NAME[] = "yolo_depth_interfaces/msg/ObjectDepthArray";
static char yolo_depth_interfaces__msg__ObjectDepth__TYPE_NAME[] = "yolo_depth_interfaces/msg/ObjectDepth";

// Define type names, field names, and default values
static char yolo_depth_interfaces__msg__ObjectDepthArray__FIELD_NAME__objects[] = "objects";

static rosidl_runtime_c__type_description__Field yolo_depth_interfaces__msg__ObjectDepthArray__FIELDS[] = {
  {
    {yolo_depth_interfaces__msg__ObjectDepthArray__FIELD_NAME__objects, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {yolo_depth_interfaces__msg__ObjectDepth__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription yolo_depth_interfaces__msg__ObjectDepthArray__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {yolo_depth_interfaces__msg__ObjectDepth__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
yolo_depth_interfaces__msg__ObjectDepthArray__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {yolo_depth_interfaces__msg__ObjectDepthArray__TYPE_NAME, 42, 42},
      {yolo_depth_interfaces__msg__ObjectDepthArray__FIELDS, 1, 1},
    },
    {yolo_depth_interfaces__msg__ObjectDepthArray__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&yolo_depth_interfaces__msg__ObjectDepth__EXPECTED_HASH, yolo_depth_interfaces__msg__ObjectDepth__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = yolo_depth_interfaces__msg__ObjectDepth__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "yolo_depth_interfaces/ObjectDepth[] objects";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
yolo_depth_interfaces__msg__ObjectDepthArray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {yolo_depth_interfaces__msg__ObjectDepthArray__TYPE_NAME, 42, 42},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 43, 43},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
yolo_depth_interfaces__msg__ObjectDepthArray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *yolo_depth_interfaces__msg__ObjectDepthArray__get_individual_type_description_source(NULL),
    sources[1] = *yolo_depth_interfaces__msg__ObjectDepth__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
