// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_trials:msg/BasePlan.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_TRIALS__MSG__DETAIL__BASE_PLAN__BUILDER_HPP_
#define ROBOT_TRIALS__MSG__DETAIL__BASE_PLAN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_trials/msg/detail/base_plan__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_trials
{

namespace msg
{

namespace builder
{

class Init_BasePlan_pos
{
public:
  Init_BasePlan_pos()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_trials::msg::BasePlan pos(::robot_trials::msg::BasePlan::_pos_type arg)
  {
    msg_.pos = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_trials::msg::BasePlan msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_trials::msg::BasePlan>()
{
  return robot_trials::msg::builder::Init_BasePlan_pos();
}

}  // namespace robot_trials

#endif  // ROBOT_TRIALS__MSG__DETAIL__BASE_PLAN__BUILDER_HPP_
