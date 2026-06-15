// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from skyvolt_msgs:msg/TrackState.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__TRACK_STATE__STRUCT_HPP_
#define SKYVOLT_MSGS__MSG__DETAIL__TRACK_STATE__STRUCT_HPP_

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
# define DEPRECATED__skyvolt_msgs__msg__TrackState __attribute__((deprecated))
#else
# define DEPRECATED__skyvolt_msgs__msg__TrackState __declspec(deprecated)
#endif

namespace skyvolt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrackState_
{
  using Type = TrackState_<ContainerAllocator>;

  explicit TrackState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->branch_id = 0ul;
      this->arclength_m = 0.0;
      this->speed_mps = 0.0;
      this->arclength_var = 0.0;
    }
  }

  explicit TrackState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    robot_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->branch_id = 0ul;
      this->arclength_m = 0.0;
      this->speed_mps = 0.0;
      this->arclength_var = 0.0;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_id_type robot_id;
  using _branch_id_type =
    uint32_t;
  _branch_id_type branch_id;
  using _arclength_m_type =
    double;
  _arclength_m_type arclength_m;
  using _speed_mps_type =
    double;
  _speed_mps_type speed_mps;
  using _arclength_var_type =
    double;
  _arclength_var_type arclength_var;

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
  Type & set__branch_id(
    const uint32_t & _arg)
  {
    this->branch_id = _arg;
    return *this;
  }
  Type & set__arclength_m(
    const double & _arg)
  {
    this->arclength_m = _arg;
    return *this;
  }
  Type & set__speed_mps(
    const double & _arg)
  {
    this->speed_mps = _arg;
    return *this;
  }
  Type & set__arclength_var(
    const double & _arg)
  {
    this->arclength_var = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    skyvolt_msgs::msg::TrackState_<ContainerAllocator> *;
  using ConstRawPtr =
    const skyvolt_msgs::msg::TrackState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<skyvolt_msgs::msg::TrackState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<skyvolt_msgs::msg::TrackState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      skyvolt_msgs::msg::TrackState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<skyvolt_msgs::msg::TrackState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      skyvolt_msgs::msg::TrackState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<skyvolt_msgs::msg::TrackState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<skyvolt_msgs::msg::TrackState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<skyvolt_msgs::msg::TrackState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__skyvolt_msgs__msg__TrackState
    std::shared_ptr<skyvolt_msgs::msg::TrackState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__skyvolt_msgs__msg__TrackState
    std::shared_ptr<skyvolt_msgs::msg::TrackState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackState_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->branch_id != other.branch_id) {
      return false;
    }
    if (this->arclength_m != other.arclength_m) {
      return false;
    }
    if (this->speed_mps != other.speed_mps) {
      return false;
    }
    if (this->arclength_var != other.arclength_var) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackState_

// alias to use template instance with default allocator
using TrackState =
  skyvolt_msgs::msg::TrackState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace skyvolt_msgs

#endif  // SKYVOLT_MSGS__MSG__DETAIL__TRACK_STATE__STRUCT_HPP_
