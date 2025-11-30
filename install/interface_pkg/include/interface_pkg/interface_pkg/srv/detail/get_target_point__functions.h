// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from interface_pkg:srv/GetTargetPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interface_pkg/srv/get_target_point.h"


#ifndef INTERFACE_PKG__SRV__DETAIL__GET_TARGET_POINT__FUNCTIONS_H_
#define INTERFACE_PKG__SRV__DETAIL__GET_TARGET_POINT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "interface_pkg/msg/rosidl_generator_c__visibility_control.h"

#include "interface_pkg/srv/detail/get_target_point__struct.h"

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_type_hash_t *
interface_pkg__srv__GetTargetPoint__get_type_hash(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_runtime_c__type_description__TypeDescription *
interface_pkg__srv__GetTargetPoint__get_type_description(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_runtime_c__type_description__TypeSource *
interface_pkg__srv__GetTargetPoint__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_runtime_c__type_description__TypeSource__Sequence *
interface_pkg__srv__GetTargetPoint__get_type_description_sources(
  const rosidl_service_type_support_t * type_support);

/// Initialize srv/GetTargetPoint message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * interface_pkg__srv__GetTargetPoint_Request
 * )) before or use
 * interface_pkg__srv__GetTargetPoint_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Request__init(interface_pkg__srv__GetTargetPoint_Request * msg);

/// Finalize srv/GetTargetPoint message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
void
interface_pkg__srv__GetTargetPoint_Request__fini(interface_pkg__srv__GetTargetPoint_Request * msg);

/// Create srv/GetTargetPoint message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * interface_pkg__srv__GetTargetPoint_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
interface_pkg__srv__GetTargetPoint_Request *
interface_pkg__srv__GetTargetPoint_Request__create(void);

/// Destroy srv/GetTargetPoint message.
/**
 * It calls
 * interface_pkg__srv__GetTargetPoint_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
void
interface_pkg__srv__GetTargetPoint_Request__destroy(interface_pkg__srv__GetTargetPoint_Request * msg);

/// Check for srv/GetTargetPoint message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Request__are_equal(const interface_pkg__srv__GetTargetPoint_Request * lhs, const interface_pkg__srv__GetTargetPoint_Request * rhs);

/// Copy a srv/GetTargetPoint message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Request__copy(
  const interface_pkg__srv__GetTargetPoint_Request * input,
  interface_pkg__srv__GetTargetPoint_Request * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_type_hash_t *
interface_pkg__srv__GetTargetPoint_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_runtime_c__type_description__TypeDescription *
interface_pkg__srv__GetTargetPoint_Request__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_runtime_c__type_description__TypeSource *
interface_pkg__srv__GetTargetPoint_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_runtime_c__type_description__TypeSource__Sequence *
interface_pkg__srv__GetTargetPoint_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/GetTargetPoint messages.
/**
 * It allocates the memory for the number of elements and calls
 * interface_pkg__srv__GetTargetPoint_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Request__Sequence__init(interface_pkg__srv__GetTargetPoint_Request__Sequence * array, size_t size);

/// Finalize array of srv/GetTargetPoint messages.
/**
 * It calls
 * interface_pkg__srv__GetTargetPoint_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
void
interface_pkg__srv__GetTargetPoint_Request__Sequence__fini(interface_pkg__srv__GetTargetPoint_Request__Sequence * array);

/// Create array of srv/GetTargetPoint messages.
/**
 * It allocates the memory for the array and calls
 * interface_pkg__srv__GetTargetPoint_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
interface_pkg__srv__GetTargetPoint_Request__Sequence *
interface_pkg__srv__GetTargetPoint_Request__Sequence__create(size_t size);

/// Destroy array of srv/GetTargetPoint messages.
/**
 * It calls
 * interface_pkg__srv__GetTargetPoint_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
void
interface_pkg__srv__GetTargetPoint_Request__Sequence__destroy(interface_pkg__srv__GetTargetPoint_Request__Sequence * array);

/// Check for srv/GetTargetPoint message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Request__Sequence__are_equal(const interface_pkg__srv__GetTargetPoint_Request__Sequence * lhs, const interface_pkg__srv__GetTargetPoint_Request__Sequence * rhs);

/// Copy an array of srv/GetTargetPoint messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Request__Sequence__copy(
  const interface_pkg__srv__GetTargetPoint_Request__Sequence * input,
  interface_pkg__srv__GetTargetPoint_Request__Sequence * output);

/// Initialize srv/GetTargetPoint message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * interface_pkg__srv__GetTargetPoint_Response
 * )) before or use
 * interface_pkg__srv__GetTargetPoint_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Response__init(interface_pkg__srv__GetTargetPoint_Response * msg);

/// Finalize srv/GetTargetPoint message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
void
interface_pkg__srv__GetTargetPoint_Response__fini(interface_pkg__srv__GetTargetPoint_Response * msg);

/// Create srv/GetTargetPoint message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * interface_pkg__srv__GetTargetPoint_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
interface_pkg__srv__GetTargetPoint_Response *
interface_pkg__srv__GetTargetPoint_Response__create(void);

/// Destroy srv/GetTargetPoint message.
/**
 * It calls
 * interface_pkg__srv__GetTargetPoint_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
void
interface_pkg__srv__GetTargetPoint_Response__destroy(interface_pkg__srv__GetTargetPoint_Response * msg);

/// Check for srv/GetTargetPoint message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Response__are_equal(const interface_pkg__srv__GetTargetPoint_Response * lhs, const interface_pkg__srv__GetTargetPoint_Response * rhs);

/// Copy a srv/GetTargetPoint message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Response__copy(
  const interface_pkg__srv__GetTargetPoint_Response * input,
  interface_pkg__srv__GetTargetPoint_Response * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_type_hash_t *
interface_pkg__srv__GetTargetPoint_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_runtime_c__type_description__TypeDescription *
interface_pkg__srv__GetTargetPoint_Response__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_runtime_c__type_description__TypeSource *
interface_pkg__srv__GetTargetPoint_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_runtime_c__type_description__TypeSource__Sequence *
interface_pkg__srv__GetTargetPoint_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/GetTargetPoint messages.
/**
 * It allocates the memory for the number of elements and calls
 * interface_pkg__srv__GetTargetPoint_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Response__Sequence__init(interface_pkg__srv__GetTargetPoint_Response__Sequence * array, size_t size);

/// Finalize array of srv/GetTargetPoint messages.
/**
 * It calls
 * interface_pkg__srv__GetTargetPoint_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
void
interface_pkg__srv__GetTargetPoint_Response__Sequence__fini(interface_pkg__srv__GetTargetPoint_Response__Sequence * array);

/// Create array of srv/GetTargetPoint messages.
/**
 * It allocates the memory for the array and calls
 * interface_pkg__srv__GetTargetPoint_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
interface_pkg__srv__GetTargetPoint_Response__Sequence *
interface_pkg__srv__GetTargetPoint_Response__Sequence__create(size_t size);

/// Destroy array of srv/GetTargetPoint messages.
/**
 * It calls
 * interface_pkg__srv__GetTargetPoint_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
void
interface_pkg__srv__GetTargetPoint_Response__Sequence__destroy(interface_pkg__srv__GetTargetPoint_Response__Sequence * array);

/// Check for srv/GetTargetPoint message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Response__Sequence__are_equal(const interface_pkg__srv__GetTargetPoint_Response__Sequence * lhs, const interface_pkg__srv__GetTargetPoint_Response__Sequence * rhs);

/// Copy an array of srv/GetTargetPoint messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Response__Sequence__copy(
  const interface_pkg__srv__GetTargetPoint_Response__Sequence * input,
  interface_pkg__srv__GetTargetPoint_Response__Sequence * output);

/// Initialize srv/GetTargetPoint message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * interface_pkg__srv__GetTargetPoint_Event
 * )) before or use
 * interface_pkg__srv__GetTargetPoint_Event__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Event__init(interface_pkg__srv__GetTargetPoint_Event * msg);

/// Finalize srv/GetTargetPoint message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
void
interface_pkg__srv__GetTargetPoint_Event__fini(interface_pkg__srv__GetTargetPoint_Event * msg);

/// Create srv/GetTargetPoint message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * interface_pkg__srv__GetTargetPoint_Event__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
interface_pkg__srv__GetTargetPoint_Event *
interface_pkg__srv__GetTargetPoint_Event__create(void);

/// Destroy srv/GetTargetPoint message.
/**
 * It calls
 * interface_pkg__srv__GetTargetPoint_Event__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
void
interface_pkg__srv__GetTargetPoint_Event__destroy(interface_pkg__srv__GetTargetPoint_Event * msg);

/// Check for srv/GetTargetPoint message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Event__are_equal(const interface_pkg__srv__GetTargetPoint_Event * lhs, const interface_pkg__srv__GetTargetPoint_Event * rhs);

/// Copy a srv/GetTargetPoint message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Event__copy(
  const interface_pkg__srv__GetTargetPoint_Event * input,
  interface_pkg__srv__GetTargetPoint_Event * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_type_hash_t *
interface_pkg__srv__GetTargetPoint_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_runtime_c__type_description__TypeDescription *
interface_pkg__srv__GetTargetPoint_Event__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_runtime_c__type_description__TypeSource *
interface_pkg__srv__GetTargetPoint_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
const rosidl_runtime_c__type_description__TypeSource__Sequence *
interface_pkg__srv__GetTargetPoint_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/GetTargetPoint messages.
/**
 * It allocates the memory for the number of elements and calls
 * interface_pkg__srv__GetTargetPoint_Event__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Event__Sequence__init(interface_pkg__srv__GetTargetPoint_Event__Sequence * array, size_t size);

/// Finalize array of srv/GetTargetPoint messages.
/**
 * It calls
 * interface_pkg__srv__GetTargetPoint_Event__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
void
interface_pkg__srv__GetTargetPoint_Event__Sequence__fini(interface_pkg__srv__GetTargetPoint_Event__Sequence * array);

/// Create array of srv/GetTargetPoint messages.
/**
 * It allocates the memory for the array and calls
 * interface_pkg__srv__GetTargetPoint_Event__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
interface_pkg__srv__GetTargetPoint_Event__Sequence *
interface_pkg__srv__GetTargetPoint_Event__Sequence__create(size_t size);

/// Destroy array of srv/GetTargetPoint messages.
/**
 * It calls
 * interface_pkg__srv__GetTargetPoint_Event__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
void
interface_pkg__srv__GetTargetPoint_Event__Sequence__destroy(interface_pkg__srv__GetTargetPoint_Event__Sequence * array);

/// Check for srv/GetTargetPoint message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Event__Sequence__are_equal(const interface_pkg__srv__GetTargetPoint_Event__Sequence * lhs, const interface_pkg__srv__GetTargetPoint_Event__Sequence * rhs);

/// Copy an array of srv/GetTargetPoint messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_pkg
bool
interface_pkg__srv__GetTargetPoint_Event__Sequence__copy(
  const interface_pkg__srv__GetTargetPoint_Event__Sequence * input,
  interface_pkg__srv__GetTargetPoint_Event__Sequence * output);
#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_PKG__SRV__DETAIL__GET_TARGET_POINT__FUNCTIONS_H_
