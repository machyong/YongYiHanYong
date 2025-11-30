// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from yolo_depth_interfaces:msg/ObjectDepth.idl
// generated code does not contain a copyright notice

#include "yolo_depth_interfaces/msg/detail/object_depth__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_yolo_depth_interfaces
const rosidl_type_hash_t *
yolo_depth_interfaces__msg__ObjectDepth__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb2, 0xd6, 0x43, 0x9b, 0x03, 0x9c, 0x0f, 0xe4,
      0xca, 0xa4, 0x36, 0x8c, 0xb1, 0x4d, 0x2a, 0x56,
      0x2f, 0x56, 0x35, 0x46, 0x3f, 0x0b, 0x08, 0x40,
      0xd9, 0x37, 0x6e, 0x64, 0x59, 0x0f, 0xd0, 0x8b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char yolo_depth_interfaces__msg__ObjectDepth__TYPE_NAME[] = "yolo_depth_interfaces/msg/ObjectDepth";

// Define type names, field names, and default values
static char yolo_depth_interfaces__msg__ObjectDepth__FIELD_NAME__class_name[] = "class_name";
static char yolo_depth_interfaces__msg__ObjectDepth__FIELD_NAME__confidence[] = "confidence";
static char yolo_depth_interfaces__msg__ObjectDepth__FIELD_NAME__distance_m[] = "distance_m";
static char yolo_depth_interfaces__msg__ObjectDepth__FIELD_NAME__x1[] = "x1";
static char yolo_depth_interfaces__msg__ObjectDepth__FIELD_NAME__y1[] = "y1";
static char yolo_depth_interfaces__msg__ObjectDepth__FIELD_NAME__x2[] = "x2";
static char yolo_depth_interfaces__msg__ObjectDepth__FIELD_NAME__y2[] = "y2";

static rosidl_runtime_c__type_description__Field yolo_depth_interfaces__msg__ObjectDepth__FIELDS[] = {
  {
    {yolo_depth_interfaces__msg__ObjectDepth__FIELD_NAME__class_name, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__msg__ObjectDepth__FIELD_NAME__confidence, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__msg__ObjectDepth__FIELD_NAME__distance_m, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__msg__ObjectDepth__FIELD_NAME__x1, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__msg__ObjectDepth__FIELD_NAME__y1, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__msg__ObjectDepth__FIELD_NAME__x2, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__msg__ObjectDepth__FIELD_NAME__y2, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
yolo_depth_interfaces__msg__ObjectDepth__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {yolo_depth_interfaces__msg__ObjectDepth__TYPE_NAME, 37, 37},
      {yolo_depth_interfaces__msg__ObjectDepth__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string class_name\n"
  "float32 confidence\n"
  "float32 distance_m\n"
  "int32 x1\n"
  "int32 y1\n"
  "int32 x2\n"
  "int32 y2";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
yolo_depth_interfaces__msg__ObjectDepth__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {yolo_depth_interfaces__msg__ObjectDepth__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 91, 91},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
yolo_depth_interfaces__msg__ObjectDepth__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *yolo_depth_interfaces__msg__ObjectDepth__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
