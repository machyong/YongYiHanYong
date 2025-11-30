// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from interface_pkg:srv/StringToString.idl
// generated code does not contain a copyright notice

#include "interface_pkg/srv/detail/string_to_string__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_type_hash_t *
interface_pkg__srv__StringToString__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe5, 0xe9, 0x79, 0x3c, 0xbb, 0x5b, 0x36, 0x90,
      0x92, 0x08, 0x7c, 0xbd, 0xcc, 0x42, 0x41, 0xca,
      0xac, 0xdf, 0x20, 0xd3, 0xd6, 0x2e, 0x6c, 0x0c,
      0xce, 0xa7, 0x29, 0x69, 0xe0, 0x58, 0x99, 0x16,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_type_hash_t *
interface_pkg__srv__StringToString_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x61, 0x31, 0xdf, 0xa5, 0xad, 0xaf, 0x9f, 0x2f,
      0x5b, 0x56, 0x7d, 0x82, 0x1c, 0xda, 0x26, 0xb6,
      0xc6, 0x20, 0x97, 0xd9, 0xac, 0x8e, 0xfc, 0xb8,
      0x02, 0x7d, 0x29, 0xa0, 0x8b, 0x91, 0x55, 0x4b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_type_hash_t *
interface_pkg__srv__StringToString_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xca, 0x33, 0xe6, 0x9d, 0x24, 0xc0, 0x3c, 0x2e,
      0x1e, 0xd6, 0xf3, 0xfd, 0x33, 0xf8, 0xcc, 0xfa,
      0x82, 0x6b, 0x6d, 0xde, 0x77, 0x55, 0xb2, 0xaa,
      0xc4, 0xe3, 0xa9, 0x6d, 0xf1, 0xe5, 0x35, 0x8a,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_type_hash_t *
interface_pkg__srv__StringToString_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x06, 0xb9, 0x7e, 0x6b, 0x6f, 0x01, 0xfe, 0x2f,
      0x47, 0xa3, 0x09, 0x99, 0x78, 0x50, 0x68, 0x42,
      0x82, 0x18, 0x95, 0x77, 0x0c, 0x1c, 0x6d, 0xeb,
      0xac, 0x7a, 0x46, 0x8f, 0xda, 0xe5, 0xfc, 0xa5,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char interface_pkg__srv__StringToString__TYPE_NAME[] = "interface_pkg/srv/StringToString";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char interface_pkg__srv__StringToString_Event__TYPE_NAME[] = "interface_pkg/srv/StringToString_Event";
static char interface_pkg__srv__StringToString_Request__TYPE_NAME[] = "interface_pkg/srv/StringToString_Request";
static char interface_pkg__srv__StringToString_Response__TYPE_NAME[] = "interface_pkg/srv/StringToString_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char interface_pkg__srv__StringToString__FIELD_NAME__request_message[] = "request_message";
static char interface_pkg__srv__StringToString__FIELD_NAME__response_message[] = "response_message";
static char interface_pkg__srv__StringToString__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field interface_pkg__srv__StringToString__FIELDS[] = {
  {
    {interface_pkg__srv__StringToString__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {interface_pkg__srv__StringToString_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__StringToString__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {interface_pkg__srv__StringToString_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__StringToString__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {interface_pkg__srv__StringToString_Event__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription interface_pkg__srv__StringToString__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__StringToString_Event__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__StringToString_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__StringToString_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
interface_pkg__srv__StringToString__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interface_pkg__srv__StringToString__TYPE_NAME, 32, 32},
      {interface_pkg__srv__StringToString__FIELDS, 3, 3},
    },
    {interface_pkg__srv__StringToString__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = interface_pkg__srv__StringToString_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = interface_pkg__srv__StringToString_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = interface_pkg__srv__StringToString_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char interface_pkg__srv__StringToString_Request__FIELD_NAME__input[] = "input";

static rosidl_runtime_c__type_description__Field interface_pkg__srv__StringToString_Request__FIELDS[] = {
  {
    {interface_pkg__srv__StringToString_Request__FIELD_NAME__input, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
interface_pkg__srv__StringToString_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interface_pkg__srv__StringToString_Request__TYPE_NAME, 40, 40},
      {interface_pkg__srv__StringToString_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char interface_pkg__srv__StringToString_Response__FIELD_NAME__output[] = "output";

static rosidl_runtime_c__type_description__Field interface_pkg__srv__StringToString_Response__FIELDS[] = {
  {
    {interface_pkg__srv__StringToString_Response__FIELD_NAME__output, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
interface_pkg__srv__StringToString_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interface_pkg__srv__StringToString_Response__TYPE_NAME, 41, 41},
      {interface_pkg__srv__StringToString_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char interface_pkg__srv__StringToString_Event__FIELD_NAME__info[] = "info";
static char interface_pkg__srv__StringToString_Event__FIELD_NAME__request[] = "request";
static char interface_pkg__srv__StringToString_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field interface_pkg__srv__StringToString_Event__FIELDS[] = {
  {
    {interface_pkg__srv__StringToString_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__StringToString_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {interface_pkg__srv__StringToString_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__StringToString_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {interface_pkg__srv__StringToString_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription interface_pkg__srv__StringToString_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__StringToString_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {interface_pkg__srv__StringToString_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
interface_pkg__srv__StringToString_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interface_pkg__srv__StringToString_Event__TYPE_NAME, 38, 38},
      {interface_pkg__srv__StringToString_Event__FIELDS, 3, 3},
    },
    {interface_pkg__srv__StringToString_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = interface_pkg__srv__StringToString_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = interface_pkg__srv__StringToString_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string input\n"
  "---\n"
  "string output";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
interface_pkg__srv__StringToString__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interface_pkg__srv__StringToString__TYPE_NAME, 32, 32},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 31, 31},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
interface_pkg__srv__StringToString_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interface_pkg__srv__StringToString_Request__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
interface_pkg__srv__StringToString_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interface_pkg__srv__StringToString_Response__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
interface_pkg__srv__StringToString_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interface_pkg__srv__StringToString_Event__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interface_pkg__srv__StringToString__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interface_pkg__srv__StringToString__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *interface_pkg__srv__StringToString_Event__get_individual_type_description_source(NULL);
    sources[3] = *interface_pkg__srv__StringToString_Request__get_individual_type_description_source(NULL);
    sources[4] = *interface_pkg__srv__StringToString_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interface_pkg__srv__StringToString_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interface_pkg__srv__StringToString_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interface_pkg__srv__StringToString_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interface_pkg__srv__StringToString_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interface_pkg__srv__StringToString_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interface_pkg__srv__StringToString_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *interface_pkg__srv__StringToString_Request__get_individual_type_description_source(NULL);
    sources[3] = *interface_pkg__srv__StringToString_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
