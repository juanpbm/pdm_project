// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_trials:msg/BasePlan.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_TRIALS__MSG__DETAIL__BASE_PLAN__TRAITS_HPP_
#define ROBOT_TRIALS__MSG__DETAIL__BASE_PLAN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_trials/msg/detail/base_plan__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_trials
{

namespace msg
{

inline void to_flow_style_yaml(
  const BasePlan & msg,
  std::ostream & out)
{
  out << "{";
  // member: pos
  {
    if (msg.pos.size() == 0) {
      out << "pos: []";
    } else {
      out << "pos: [";
      size_t pending_items = msg.pos.size();
      for (auto item : msg.pos) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BasePlan & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pos.size() == 0) {
      out << "pos: []\n";
    } else {
      out << "pos:\n";
      for (auto item : msg.pos) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BasePlan & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robot_trials

namespace rosidl_generator_traits
{

[[deprecated("use robot_trials::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_trials::msg::BasePlan & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_trials::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_trials::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_trials::msg::BasePlan & msg)
{
  return robot_trials::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_trials::msg::BasePlan>()
{
  return "robot_trials::msg::BasePlan";
}

template<>
inline const char * name<robot_trials::msg::BasePlan>()
{
  return "robot_trials/msg/BasePlan";
}

template<>
struct has_fixed_size<robot_trials::msg::BasePlan>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_trials::msg::BasePlan>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_trials::msg::BasePlan>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_TRIALS__MSG__DETAIL__BASE_PLAN__TRAITS_HPP_
