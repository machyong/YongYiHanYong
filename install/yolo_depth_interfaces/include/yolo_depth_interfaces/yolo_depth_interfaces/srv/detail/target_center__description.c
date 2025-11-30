// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from yolo_depth_interfaces:srv/TargetCenter.idl
// generated code does not contain a copyright notice

#include "yolo_depth_interfaces/srv/detail/target_center__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_yolo_depth_interfaces
const rosidl_type_hash_t *
yolo_depth_interfaces__srv__TargetCenter__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd8, 0xff, 0x47, 0xda, 0xc2, 0x39, 0x17, 0xac,
      0x90, 0xf3, 0x04, 0xd0, 0xb2, 0x91, 0x62, 0x7e,
      0x9e, 0xb2, 0xc4, 0xe4, 0x9c, 0xc1, 0x85, 0x52,
      0xe6, 0xf9, 0xf6, 0x37, 0x3a, 0xf9, 0xf0, 0x7d,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_yolo_depth_interfaces
const rosidl_type_hash_t *
yolo_depth_interfaces__srv__TargetCenter_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x70, 0xcd, 0xd2, 0x09, 0x52, 0xe6, 0xf7, 0x59,
      0xdc, 0x33, 0xad, 0x8f, 0xd0, 0xfa, 0x2b, 0xee,
      0xd7, 0x4c, 0xf1, 0x64, 0x64, 0x4e, 0x05, 0x92,
      0x7b, 0x9a, 0x47, 0xbe, 0x6f, 0x76, 0xc2, 0xbe,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_yolo_depth_interfaces
const rosidl_type_hash_t *
yolo_depth_interfaces__srv__TargetCenter_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc4, 0x80, 0x9a, 0x82, 0xda, 0xa6, 0xba, 0x53,
      0x1c, 0xfd, 0xbe, 0xe6, 0xcf, 0x7b, 0x7b, 0x89,
      0x23, 0x2b, 0x98, 0x70, 0x68, 0x01, 0xf9, 0x03,
      0xaa, 0x92, 0x17, 0xbf, 0xaf, 0xd5, 0x33, 0xfd,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_yolo_depth_interfaces
const rosidl_type_hash_t *
yolo_depth_interfaces__srv__TargetCenter_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbf, 0x22, 0x2d, 0x03, 0x64, 0x23, 0x1d, 0x55,
      0xe7, 0x07, 0x21, 0xd1, 0xbd, 0x6d, 0x6b, 0x99,
      0xfb, 0x98, 0xf9, 0x51, 0xd9, 0xba, 0x75, 0xb5,
      0x20, 0x88, 0x42, 0x51, 0xe1, 0x9a, 0x43, 0x26,
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

static char yolo_depth_interfaces__srv__TargetCenter__TYPE_NAME[] = "yolo_depth_interfaces/srv/TargetCenter";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char yolo_depth_interfaces__srv__TargetCenter_Event__TYPE_NAME[] = "yolo_depth_interfaces/srv/TargetCenter_Event";
static char yolo_depth_interfaces__srv__TargetCenter_Request__TYPE_NAME[] = "yolo_depth_interfaces/srv/TargetCenter_Request";
static char yolo_depth_interfaces__srv__TargetCenter_Response__TYPE_NAME[] = "yolo_depth_interfaces/srv/TargetCenter_Response";

// Define type names, field names, and default values
static char yolo_depth_interfaces__srv__TargetCenter__FIELD_NAME__request_message[] = "request_message";
static char yolo_depth_interfaces__srv__TargetCenter__FIELD_NAME__response_message[] = "response_message";
static char yolo_depth_interfaces__srv__TargetCenter__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field yolo_depth_interfaces__srv__TargetCenter__FIELDS[] = {
  {
    {yolo_depth_interfaces__srv__TargetCenter__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {yolo_depth_interfaces__srv__TargetCenter_Request__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__srv__TargetCenter__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {yolo_depth_interfaces__srv__TargetCenter_Response__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__srv__TargetCenter__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {yolo_depth_interfaces__srv__TargetCenter_Event__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription yolo_depth_interfaces__srv__TargetCenter__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__srv__TargetCenter_Event__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__srv__TargetCenter_Request__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__srv__TargetCenter_Response__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
yolo_depth_interfaces__srv__TargetCenter__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {yolo_depth_interfaces__srv__TargetCenter__TYPE_NAME, 38, 38},
      {yolo_depth_interfaces__srv__TargetCenter__FIELDS, 3, 3},
    },
    {yolo_depth_interfaces__srv__TargetCenter__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = yolo_depth_interfaces__srv__TargetCenter_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = yolo_depth_interfaces__srv__TargetCenter_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = yolo_depth_interfaces__srv__TargetCenter_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char yolo_depth_interfaces__srv__TargetCenter_Request__FIELD_NAME__target[] = "target";
static char yolo_depth_interfaces__srv__TargetCenter_Request__FIELD_NAME__center_x[] = "center_x";
static char yolo_depth_interfaces__srv__TargetCenter_Request__FIELD_NAME__center_y[] = "center_y";
static char yolo_depth_interfaces__srv__TargetCenter_Request__FIELD_NAME__distance_m[] = "distance_m";

static rosidl_runtime_c__type_description__Field yolo_depth_interfaces__srv__TargetCenter_Request__FIELDS[] = {
  {
    {yolo_depth_interfaces__srv__TargetCenter_Request__FIELD_NAME__target, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__srv__TargetCenter_Request__FIELD_NAME__center_x, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__srv__TargetCenter_Request__FIELD_NAME__center_y, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__srv__TargetCenter_Request__FIELD_NAME__distance_m, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
yolo_depth_interfaces__srv__TargetCenter_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {yolo_depth_interfaces__srv__TargetCenter_Request__TYPE_NAME, 46, 46},
      {yolo_depth_interfaces__srv__TargetCenter_Request__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char yolo_depth_interfaces__srv__TargetCenter_Response__FIELD_NAME__success[] = "success";
static char yolo_depth_interfaces__srv__TargetCenter_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field yolo_depth_interfaces__srv__TargetCenter_Response__FIELDS[] = {
  {
    {yolo_depth_interfaces__srv__TargetCenter_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__srv__TargetCenter_Response__FIELD_NAME__message, 7, 7},
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
yolo_depth_interfaces__srv__TargetCenter_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {yolo_depth_interfaces__srv__TargetCenter_Response__TYPE_NAME, 47, 47},
      {yolo_depth_interfaces__srv__TargetCenter_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char yolo_depth_interfaces__srv__TargetCenter_Event__FIELD_NAME__info[] = "info";
static char yolo_depth_interfaces__srv__TargetCenter_Event__FIELD_NAME__request[] = "request";
static char yolo_depth_interfaces__srv__TargetCenter_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field yolo_depth_interfaces__srv__TargetCenter_Event__FIELDS[] = {
  {
    {yolo_depth_interfaces__srv__TargetCenter_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__srv__TargetCenter_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {yolo_depth_interfaces__srv__TargetCenter_Request__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__srv__TargetCenter_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {yolo_depth_interfaces__srv__TargetCenter_Response__TYPE_NAME, 47, 47},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription yolo_depth_interfaces__srv__TargetCenter_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__srv__TargetCenter_Request__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {yolo_depth_interfaces__srv__TargetCenter_Response__TYPE_NAME, 47, 47},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
yolo_depth_interfaces__srv__TargetCenter_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {yolo_depth_interfaces__srv__TargetCenter_Event__TYPE_NAME, 44, 44},
      {yolo_depth_interfaces__srv__TargetCenter_Event__FIELDS, 3, 3},
    },
    {yolo_depth_interfaces__srv__TargetCenter_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = yolo_depth_interfaces__srv__TargetCenter_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = yolo_depth_interfaces__srv__TargetCenter_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Request: \\xec\\xa4\\x91\\xec\\x8b\\xac\\xec\\xa0\\x90 \\xeb\\x8d\\xb0\\xec\\x9d\\xb4\\xed\\x84\\xb0 \\xec\\xa0\\x84\\xec\\x86\\xa1\n"
  "string target\n"
  "float64 center_x\n"
  "float64 center_y\n"
  "float64 distance_m\n"
  "---\n"
  "# Response: \\xec\\x88\\x98\\xec\\x8b\\xa0 \\xed\\x99\\x95\\xec\\x9d\\xb8\n"
  "bool success\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
yolo_depth_interfaces__srv__TargetCenter__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {yolo_depth_interfaces__srv__TargetCenter__TYPE_NAME, 38, 38},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 139, 139},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
yolo_depth_interfaces__srv__TargetCenter_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {yolo_depth_interfaces__srv__TargetCenter_Request__TYPE_NAME, 46, 46},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
yolo_depth_interfaces__srv__TargetCenter_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {yolo_depth_interfaces__srv__TargetCenter_Response__TYPE_NAME, 47, 47},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
yolo_depth_interfaces__srv__TargetCenter_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {yolo_depth_interfaces__srv__TargetCenter_Event__TYPE_NAME, 44, 44},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
yolo_depth_interfaces__srv__TargetCenter__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *yolo_depth_interfaces__srv__TargetCenter__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *yolo_depth_interfaces__srv__TargetCenter_Event__get_individual_type_description_source(NULL);
    sources[4] = *yolo_depth_interfaces__srv__TargetCenter_Request__get_individual_type_description_source(NULL);
    sources[5] = *yolo_depth_interfaces__srv__TargetCenter_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
yolo_depth_interfaces__srv__TargetCenter_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *yolo_depth_interfaces__srv__TargetCenter_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
yolo_depth_interfaces__srv__TargetCenter_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *yolo_depth_interfaces__srv__TargetCenter_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
yolo_depth_interfaces__srv__TargetCenter_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *yolo_depth_interfaces__srv__TargetCenter_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *yolo_depth_interfaces__srv__TargetCenter_Request__get_individual_type_description_source(NULL);
    sources[4] = *yolo_depth_interfaces__srv__TargetCenter_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
