// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_trials:msg/BasePlan.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_trials/msg/detail/base_plan__rosidl_typesupport_introspection_c.h"
#include "robot_trials/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_trials/msg/detail/base_plan__functions.h"
#include "robot_trials/msg/detail/base_plan__struct.h"


// Include directives for member types
// Member `pos`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__BasePlan_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_trials__msg__BasePlan__init(message_memory);
}

void robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__BasePlan_fini_function(void * message_memory)
{
  robot_trials__msg__BasePlan__fini(message_memory);
}

size_t robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__size_function__BasePlan__pos(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__get_const_function__BasePlan__pos(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__get_function__BasePlan__pos(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__fetch_function__BasePlan__pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__get_const_function__BasePlan__pos(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__assign_function__BasePlan__pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__get_function__BasePlan__pos(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__resize_function__BasePlan__pos(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__BasePlan_message_member_array[1] = {
  {
    "pos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_trials__msg__BasePlan, pos),  // bytes offset in struct
    NULL,  // default value
    robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__size_function__BasePlan__pos,  // size() function pointer
    robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__get_const_function__BasePlan__pos,  // get_const(index) function pointer
    robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__get_function__BasePlan__pos,  // get(index) function pointer
    robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__fetch_function__BasePlan__pos,  // fetch(index, &value) function pointer
    robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__assign_function__BasePlan__pos,  // assign(index, value) function pointer
    robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__resize_function__BasePlan__pos  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__BasePlan_message_members = {
  "robot_trials__msg",  // message namespace
  "BasePlan",  // message name
  1,  // number of fields
  sizeof(robot_trials__msg__BasePlan),
  robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__BasePlan_message_member_array,  // message members
  robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__BasePlan_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__BasePlan_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__BasePlan_message_type_support_handle = {
  0,
  &robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__BasePlan_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_trials
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_trials, msg, BasePlan)() {
  if (!robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__BasePlan_message_type_support_handle.typesupport_identifier) {
    robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__BasePlan_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_trials__msg__BasePlan__rosidl_typesupport_introspection_c__BasePlan_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
