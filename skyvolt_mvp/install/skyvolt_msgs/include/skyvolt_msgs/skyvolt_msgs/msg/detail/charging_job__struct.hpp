// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from skyvolt_msgs:msg/ChargingJob.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__CHARGING_JOB__STRUCT_HPP_
#define SKYVOLT_MSGS__MSG__DETAIL__CHARGING_JOB__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'submitted_at'
// Member 'deadline'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__skyvolt_msgs__msg__ChargingJob __attribute__((deprecated))
#else
# define DEPRECATED__skyvolt_msgs__msg__ChargingJob __declspec(deprecated)
#endif

namespace skyvolt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ChargingJob_
{
  using Type = ChargingJob_<ContainerAllocator>;

  explicit ChargingJob_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : submitted_at(_init),
    deadline(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->job_id = "";
      this->requester_id = "";
      this->target_branch_id = 0ul;
      this->target_arclength_m = 0.0;
      this->priority = 0;
    }
  }

  explicit ChargingJob_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : job_id(_alloc),
    requester_id(_alloc),
    submitted_at(_alloc, _init),
    deadline(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->job_id = "";
      this->requester_id = "";
      this->target_branch_id = 0ul;
      this->target_arclength_m = 0.0;
      this->priority = 0;
    }
  }

  // field types and members
  using _job_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _job_id_type job_id;
  using _requester_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _requester_id_type requester_id;
  using _target_branch_id_type =
    uint32_t;
  _target_branch_id_type target_branch_id;
  using _target_arclength_m_type =
    double;
  _target_arclength_m_type target_arclength_m;
  using _submitted_at_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _submitted_at_type submitted_at;
  using _deadline_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _deadline_type deadline;
  using _priority_type =
    uint8_t;
  _priority_type priority;

  // setters for named parameter idiom
  Type & set__job_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->job_id = _arg;
    return *this;
  }
  Type & set__requester_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->requester_id = _arg;
    return *this;
  }
  Type & set__target_branch_id(
    const uint32_t & _arg)
  {
    this->target_branch_id = _arg;
    return *this;
  }
  Type & set__target_arclength_m(
    const double & _arg)
  {
    this->target_arclength_m = _arg;
    return *this;
  }
  Type & set__submitted_at(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->submitted_at = _arg;
    return *this;
  }
  Type & set__deadline(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->deadline = _arg;
    return *this;
  }
  Type & set__priority(
    const uint8_t & _arg)
  {
    this->priority = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    skyvolt_msgs::msg::ChargingJob_<ContainerAllocator> *;
  using ConstRawPtr =
    const skyvolt_msgs::msg::ChargingJob_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<skyvolt_msgs::msg::ChargingJob_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<skyvolt_msgs::msg::ChargingJob_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      skyvolt_msgs::msg::ChargingJob_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<skyvolt_msgs::msg::ChargingJob_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      skyvolt_msgs::msg::ChargingJob_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<skyvolt_msgs::msg::ChargingJob_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<skyvolt_msgs::msg::ChargingJob_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<skyvolt_msgs::msg::ChargingJob_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__skyvolt_msgs__msg__ChargingJob
    std::shared_ptr<skyvolt_msgs::msg::ChargingJob_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__skyvolt_msgs__msg__ChargingJob
    std::shared_ptr<skyvolt_msgs::msg::ChargingJob_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChargingJob_ & other) const
  {
    if (this->job_id != other.job_id) {
      return false;
    }
    if (this->requester_id != other.requester_id) {
      return false;
    }
    if (this->target_branch_id != other.target_branch_id) {
      return false;
    }
    if (this->target_arclength_m != other.target_arclength_m) {
      return false;
    }
    if (this->submitted_at != other.submitted_at) {
      return false;
    }
    if (this->deadline != other.deadline) {
      return false;
    }
    if (this->priority != other.priority) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChargingJob_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChargingJob_

// alias to use template instance with default allocator
using ChargingJob =
  skyvolt_msgs::msg::ChargingJob_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace skyvolt_msgs

#endif  // SKYVOLT_MSGS__MSG__DETAIL__CHARGING_JOB__STRUCT_HPP_
