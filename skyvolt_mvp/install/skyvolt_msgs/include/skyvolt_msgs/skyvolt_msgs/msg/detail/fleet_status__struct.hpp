// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from skyvolt_msgs:msg/FleetStatus.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__FLEET_STATUS__STRUCT_HPP_
#define SKYVOLT_MSGS__MSG__DETAIL__FLEET_STATUS__STRUCT_HPP_

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
# define DEPRECATED__skyvolt_msgs__msg__FleetStatus __attribute__((deprecated))
#else
# define DEPRECATED__skyvolt_msgs__msg__FleetStatus __declspec(deprecated)
#endif

namespace skyvolt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FleetStatus_
{
  using Type = FleetStatus_<ContainerAllocator>;

  explicit FleetStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_robots = 0ul;
      this->num_pending_jobs = 0ul;
      this->num_active_jobs = 0ul;
      this->num_reservations = 0ul;
      this->avg_job_age_s = 0.0;
    }
  }

  explicit FleetStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_robots = 0ul;
      this->num_pending_jobs = 0ul;
      this->num_active_jobs = 0ul;
      this->num_reservations = 0ul;
      this->avg_job_age_s = 0.0;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _num_robots_type =
    uint32_t;
  _num_robots_type num_robots;
  using _num_pending_jobs_type =
    uint32_t;
  _num_pending_jobs_type num_pending_jobs;
  using _num_active_jobs_type =
    uint32_t;
  _num_active_jobs_type num_active_jobs;
  using _num_reservations_type =
    uint32_t;
  _num_reservations_type num_reservations;
  using _avg_job_age_s_type =
    double;
  _avg_job_age_s_type avg_job_age_s;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__num_robots(
    const uint32_t & _arg)
  {
    this->num_robots = _arg;
    return *this;
  }
  Type & set__num_pending_jobs(
    const uint32_t & _arg)
  {
    this->num_pending_jobs = _arg;
    return *this;
  }
  Type & set__num_active_jobs(
    const uint32_t & _arg)
  {
    this->num_active_jobs = _arg;
    return *this;
  }
  Type & set__num_reservations(
    const uint32_t & _arg)
  {
    this->num_reservations = _arg;
    return *this;
  }
  Type & set__avg_job_age_s(
    const double & _arg)
  {
    this->avg_job_age_s = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    skyvolt_msgs::msg::FleetStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const skyvolt_msgs::msg::FleetStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<skyvolt_msgs::msg::FleetStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<skyvolt_msgs::msg::FleetStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      skyvolt_msgs::msg::FleetStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<skyvolt_msgs::msg::FleetStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      skyvolt_msgs::msg::FleetStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<skyvolt_msgs::msg::FleetStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<skyvolt_msgs::msg::FleetStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<skyvolt_msgs::msg::FleetStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__skyvolt_msgs__msg__FleetStatus
    std::shared_ptr<skyvolt_msgs::msg::FleetStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__skyvolt_msgs__msg__FleetStatus
    std::shared_ptr<skyvolt_msgs::msg::FleetStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FleetStatus_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->num_robots != other.num_robots) {
      return false;
    }
    if (this->num_pending_jobs != other.num_pending_jobs) {
      return false;
    }
    if (this->num_active_jobs != other.num_active_jobs) {
      return false;
    }
    if (this->num_reservations != other.num_reservations) {
      return false;
    }
    if (this->avg_job_age_s != other.avg_job_age_s) {
      return false;
    }
    return true;
  }
  bool operator!=(const FleetStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FleetStatus_

// alias to use template instance with default allocator
using FleetStatus =
  skyvolt_msgs::msg::FleetStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace skyvolt_msgs

#endif  // SKYVOLT_MSGS__MSG__DETAIL__FLEET_STATUS__STRUCT_HPP_
