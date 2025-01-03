// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_trials:msg/BasePlan.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_TRIALS__MSG__DETAIL__BASE_PLAN__STRUCT_H_
#define ROBOT_TRIALS__MSG__DETAIL__BASE_PLAN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pos'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/BasePlan in the package robot_trials.
typedef struct robot_trials__msg__BasePlan
{
  rosidl_runtime_c__double__Sequence pos;
} robot_trials__msg__BasePlan;

// Struct for a sequence of robot_trials__msg__BasePlan.
typedef struct robot_trials__msg__BasePlan__Sequence
{
  robot_trials__msg__BasePlan * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_trials__msg__BasePlan__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_TRIALS__MSG__DETAIL__BASE_PLAN__STRUCT_H_
