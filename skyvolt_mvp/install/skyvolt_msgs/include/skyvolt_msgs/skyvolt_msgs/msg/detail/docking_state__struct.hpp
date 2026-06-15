// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from skyvolt_msgs:msg/DockingState.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__DOCKING_STATE__STRUCT_HPP_
#define SKYVOLT_MSGS__MSG__DETAIL__DOCKING_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__skyvolt_msgs__msg__DockingState __attribute__((deprecated))
#else
# define DEPRECATED__skyvolt_msgs__msg__DockingState __declspec(deprecated)
#endif

namespace skyvolt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DockingState_
{
  using Type = DockingState_<ContainerAllocator>;

  explicit DockingState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->state = 0;
      this->lateral_error_mm = 0.0;
      this->longitudinal_error_mm = 0.0;
    }
  }

  explicit DockingState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    robot_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->state = 0;
      this->lateral_error_mm = 0.0;
      this->longitudinal_error_mm = 0.0;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_id_type robot_id;
  using _state_type =
    uint8_t;
  _state_type state;
  using _lateral_error_mm_type =
    double;
  _lateral_error_mm_type lateral_error_mm;
  using _longitudinal_error_mm_type =
    double;
  _longitudinal_error_mm_type longitudinal_error_mm;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__robot_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__lateral_error_mm(
    const double & _arg)
  {
    this->lateral_error_mm = _arg;
    return *this;
  }
  Type & set__longitudinal_error_mm(
    const double & _arg)
  {
    this->longitudinal_error_mm = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t STATE_IDLE =
    0u;
  static constexpr uint8_t STATE_CRUISE =
    1u;
  static constexpr uint8_t STATE_APPROACH_1 =
    2u;
  static constexpr uint8_t STATE_APPROACH_2 =
    3u;
  static constexpr uint8_t STATE_DOCKED =
    4u;
  static constexpr uint8_t STATE_FAULT =
    255u;

  // pointer types
  using RawPtr =
    skyvolt_msgs::msg::DockingState_<ContainerAllocator> *;
  using ConstRawPtr =
    const skyvolt_msgs::msg::DockingState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<skyvolt_msgs::msg::DockingState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<skyvolt_msgs::msg::DockingState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      skyvolt_msgs::msg::DockingState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<skyvolt_msgs::msg::DockingState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      skyvolt_msgs::msg::DockingState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<skyvolt_msgs::msg::DockingState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<skyvolt_msgs::msg::DockingState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<skyvolt_msgs::msg::DockingState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__skyvolt_msgs__msg__DockingState
    std::shared_ptr<skyvolt_msgs::msg::DockingState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__skyvolt_msgs__msg__DockingState
    std::shared_ptr<skyvolt_msgs::msg::DockingState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DockingState_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->lateral_error_mm != other.lateral_error_mm) {
      return false;
    }
    if (this->longitudinal_error_mm != other.longitudinal_error_mm) {
      return false;
    }
    return true;
  }
  bool operator!=(const DockingState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DockingState_

// alias to use template instance with default allocator
using DockingState =
  skyvolt_msgs::msg::DockingState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DockingState_<ContainerAllocator>::STATE_IDLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DockingState_<ContainerAllocator>::STATE_CRUISE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DockingState_<ContainerAllocator>::STATE_APPROACH_1;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DockingState_<ContainerAllocator>::STATE_APPROACH_2;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DockingState_<ContainerAllocator>::STATE_DOCKED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DockingState_<ContainerAllocator>::STATE_FAULT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace skyvolt_msgs

#endif  // SKYVOLT_MSGS__MSG__DETAIL__DOCKING_STATE__STRUCT_HPP_
