// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_trials:msg/BasePlan.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_TRIALS__MSG__DETAIL__BASE_PLAN__STRUCT_HPP_
#define ROBOT_TRIALS__MSG__DETAIL__BASE_PLAN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_trials__msg__BasePlan __attribute__((deprecated))
#else
# define DEPRECATED__robot_trials__msg__BasePlan __declspec(deprecated)
#endif

namespace robot_trials
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BasePlan_
{
  using Type = BasePlan_<ContainerAllocator>;

  explicit BasePlan_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit BasePlan_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _pos_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _pos_type pos;

  // setters for named parameter idiom
  Type & set__pos(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->pos = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_trials::msg::BasePlan_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_trials::msg::BasePlan_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_trials::msg::BasePlan_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_trials::msg::BasePlan_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_trials::msg::BasePlan_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_trials::msg::BasePlan_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_trials::msg::BasePlan_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_trials::msg::BasePlan_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_trials::msg::BasePlan_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_trials::msg::BasePlan_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_trials__msg__BasePlan
    std::shared_ptr<robot_trials::msg::BasePlan_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_trials__msg__BasePlan
    std::shared_ptr<robot_trials::msg::BasePlan_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BasePlan_ & other) const
  {
    if (this->pos != other.pos) {
      return false;
    }
    return true;
  }
  bool operator!=(const BasePlan_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BasePlan_

// alias to use template instance with default allocator
using BasePlan =
  robot_trials::msg::BasePlan_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_trials

#endif  // ROBOT_TRIALS__MSG__DETAIL__BASE_PLAN__STRUCT_HPP_
