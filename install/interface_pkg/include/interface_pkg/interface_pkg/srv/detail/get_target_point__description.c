// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from interface_pkg:srv/GetTargetPoint.idl
// generated code does not contain a copyright notice

#include "interface_pkg/srv/detail/get_target_point__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_type_hash_t *
interface_pkg__srv__GetTargetPoint__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfa, 0xdc, 0xbc, 0xea, 0xf3, 0xad, 0x55, 0x2f,
      0xa9, 0x31, 0x8a, 0xa2, 0x4c, 0x92, 0x82, 0x2e,
      0xec, 0x0e, 0x06, 0xab, 0xa0, 0x05, 0x1a, 0xf2,
      0xf4, 0xb9, 0x44, 0xde, 0xea, 0x02, 0x77, 0x14,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_type_hash_t *
interface_pkg__srv__GetTargetPoint_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa9, 0x72, 0x39, 0x1a, 0x75, 0x84, 0x6f, 0xa7,
      0x4c, 0x2d, 0xb0, 0xe2, 0xaf, 0x19, 0x79, 0xb6,
      0x0e, 0x26, 0xa5, 0x7f, 0xfd, 0xbc, 0x14, 0xf9,
      0x61, 0x31, 0xf2, 0x5e, 0x8b, 0xfe, 0x39, 0xd3,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_type_hash_t *
interface_pkg__srv__GetTargetPoint_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x46, 0x00, 0x1c, 0x38, 0x41, 0x7a, 0x34, 0xf6,
      0x0a, 0x0b, 0x34, 0x52, 0xb5, 0x90, 0xcb, 0x1b,
      0x6f, 0x37, 0x55, 0xd5, 0xc8, 0xd3, 0x18, 0x19,
      0x88, 0x67, 0x55, 0xcc, 0xb2, 0x80, 0xb9, 0xe6,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_type_hash_t *
interface_pkg__srv__GetTargetPoint_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf3, 0x7f, 0x96, 0x61, 0x2a, 0xce, 0x43, 0x3e,
      0x32, 0x2f, 0xfa, 0x34, 0xa2, 0xb8, 0x50, 0x53,
      0x53, 0x22, 0x24, 0xea, 0x91, 0xb7, 0xd9, 0x8d,
      0x92, 0xec, 0x87, 0x12, 0xa9, 0xcb, 0x25, 0x31,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "geometry_msgs/msg/detail/point__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char interface_pkg__srv__GetTargetPoint__TYPE_NAME[] = "interface_pkg/srv/GetTargetPoint";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char interface_pkg__srv__GetTargetPoint_Event__TYPE_NAME[] = "interface_pkg/srv/GetTargetPoint_Event";
static char interface_pkg__srv__GetTargetPoint_Request__TYPE_NAME[] = "interface_pkg/srv/GetTargetPoint_Request";
static char interface_pkg__srv__GetTargetPoint_Response__TYPE_NAME[] = "interface_pkg/srv/GetTargetPoint_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char interface_pkg__srv__GetTargetPoint__FIELD_NAME__request_message[] = "request_message";
static char interface_pkg__srv__GetTargetPoint__FIELD_NAME__response_message[] = "response_message";
static char interface_pkg__srv__GetTargetPoint__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field interface_pkg__srv__GetTargetPoint__FIELDS[] = {
  {
    {interface_pkg__srv__GetTargetPoint__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {interface_pkg__srv__GetTargetPoint_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__GetTargetPoint__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {interface_pkg__srv__GetTargetPoint_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__GetTargetPoint__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {interface_pkg__srv__GetTargetPoint_Event__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription interface_pkg__srv__GetTargetPoint__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__GetTargetPoint_Event__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__GetTargetPoint_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__GetTargetPoint_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
interface_pkg__srv__GetTargetPoint__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interface_pkg__srv__GetTargetPoint__TYPE_NAME, 32, 32},
      {interface_pkg__srv__GetTargetPoint__FIELDS, 3, 3},
    },
    {interface_pkg__srv__GetTargetPoint__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = interface_pkg__srv__GetTargetPoint_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = interface_pkg__srv__GetTargetPoint_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = interface_pkg__srv__GetTargetPoint_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char interface_pkg__srv__GetTargetPoint_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field interface_pkg__srv__GetTargetPoint_Request__FIELDS[] = {
  {
    {interface_pkg__srv__GetTargetPoint_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
interface_pkg__srv__GetTargetPoint_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interface_pkg__srv__GetTargetPoint_Request__TYPE_NAME, 40, 40},
      {interface_pkg__srv__GetTargetPoint_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char interface_pkg__srv__GetTargetPoint_Response__FIELD_NAME__target_point[] = "target_point";
static char interface_pkg__srv__GetTargetPoint_Response__FIELD_NAME__class_name[] = "class_name";
static char interface_pkg__srv__GetTargetPoint_Response__FIELD_NAME__confidence[] = "confidence";

static rosidl_runtime_c__type_description__Field interface_pkg__srv__GetTargetPoint_Response__FIELDS[] = {
  {
    {interface_pkg__srv__GetTargetPoint_Response__FIELD_NAME__target_point, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__GetTargetPoint_Response__FIELD_NAME__class_name, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__GetTargetPoint_Response__FIELD_NAME__confidence, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription interface_pkg__srv__GetTargetPoint_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
interface_pkg__srv__GetTargetPoint_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interface_pkg__srv__GetTargetPoint_Response__TYPE_NAME, 41, 41},
      {interface_pkg__srv__GetTargetPoint_Response__FIELDS, 3, 3},
    },
    {interface_pkg__srv__GetTargetPoint_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char interface_pkg__srv__GetTargetPoint_Event__FIELD_NAME__info[] = "info";
static char interface_pkg__srv__GetTargetPoint_Event__FIELD_NAME__request[] = "request";
static char interface_pkg__srv__GetTargetPoint_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field interface_pkg__srv__GetTargetPoint_Event__FIELDS[] = {
  {
    {interface_pkg__srv__GetTargetPoint_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__GetTargetPoint_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {interface_pkg__srv__GetTargetPoint_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__GetTargetPoint_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {interface_pkg__srv__GetTargetPoint_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription interface_pkg__srv__GetTargetPoint_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__GetTargetPoint_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__GetTargetPoint_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
interface_pkg__srv__GetTargetPoint_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interface_pkg__srv__GetTargetPoint_Event__TYPE_NAME, 38, 38},
      {interface_pkg__srv__GetTargetPoint_Event__FIELDS, 3, 3},
    },
    {interface_pkg__srv__GetTargetPoint_Event__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = interface_pkg__srv__GetTargetPoint_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = interface_pkg__srv__GetTargetPoint_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "---\n"
  "geometry_msgs/Point target_point\n"
  "string class_name\n"
  "float32 confidence";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
interface_pkg__srv__GetTargetPoint__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interface_pkg__srv__GetTargetPoint__TYPE_NAME, 32, 32},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 74, 74},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
interface_pkg__srv__GetTargetPoint_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interface_pkg__srv__GetTargetPoint_Request__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
interface_pkg__srv__GetTargetPoint_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interface_pkg__srv__GetTargetPoint_Response__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
interface_pkg__srv__GetTargetPoint_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interface_pkg__srv__GetTargetPoint_Event__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interface_pkg__srv__GetTargetPoint__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interface_pkg__srv__GetTargetPoint__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *interface_pkg__srv__GetTargetPoint_Event__get_individual_type_description_source(NULL);
    sources[4] = *interface_pkg__srv__GetTargetPoint_Request__get_individual_type_description_source(NULL);
    sources[5] = *interface_pkg__srv__GetTargetPoint_Response__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interface_pkg__srv__GetTargetPoint_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interface_pkg__srv__GetTargetPoint_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interface_pkg__srv__GetTargetPoint_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interface_pkg__srv__GetTargetPoint_Response__get_individual_type_description_source(NULL),
    sources[1] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interface_pkg__srv__GetTargetPoint_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interface_pkg__srv__GetTargetPoint_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *interface_pkg__srv__GetTargetPoint_Request__get_individual_type_description_source(NULL);
    sources[4] = *interface_pkg__srv__GetTargetPoint_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
