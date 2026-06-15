// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from skyvolt_msgs:srv/AssignJob.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__SRV__DETAIL__ASSIGN_JOB__STRUCT_HPP_
#define SKYVOLT_MSGS__SRV__DETAIL__ASSIGN_JOB__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'job'
#include "skyvolt_msgs/msg/detail/charging_job__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__skyvolt_msgs__srv__AssignJob_Request __attribute__((deprecated))
#else
# define DEPRECATED__skyvolt_msgs__srv__AssignJob_Request __declspec(deprecated)
#endif

namespace skyvolt_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct AssignJob_Request_
{
  using Type = AssignJob_Request_<ContainerAllocator>;

  explicit AssignJob_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : job(_init)
  {
    (void)_init;
  }

  explicit AssignJob_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : job(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _job_type =
    skyvolt_msgs::msg::ChargingJob_<ContainerAllocator>;
  _job_type job;

  // setters for named parameter idiom
  Type & set__job(
    const skyvolt_msgs::msg::ChargingJob_<ContainerAllocator> & _arg)
  {
    this->job = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    skyvolt_msgs::srv::AssignJob_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const skyvolt_msgs::srv::AssignJob_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<skyvolt_msgs::srv::AssignJob_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<skyvolt_msgs::srv::AssignJob_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      skyvolt_msgs::srv::AssignJob_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<skyvolt_msgs::srv::AssignJob_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      skyvolt_msgs::srv::AssignJob_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<skyvolt_msgs::srv::AssignJob_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<skyvolt_msgs::srv::AssignJob_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<skyvolt_msgs::srv::AssignJob_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__skyvolt_msgs__srv__AssignJob_Request
    std::shared_ptr<skyvolt_msgs::srv::AssignJob_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__skyvolt_msgs__srv__AssignJob_Request
    std::shared_ptr<skyvolt_msgs::srv::AssignJob_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AssignJob_Request_ & other) const
  {
    if (this->job != other.job) {
      return false;
    }
    return true;
  }
  bool operator!=(const AssignJob_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AssignJob_Request_

// alias to use template instance with default allocator
using AssignJob_Request =
  skyvolt_msgs::srv::AssignJob_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace skyvolt_msgs


#ifndef _WIN32
# define DEPRECATED__skyvolt_msgs__srv__AssignJob_Response __attribute__((deprecated))
#else
# define DEPRECATED__skyvolt_msgs__srv__AssignJob_Response __declspec(deprecated)
#endif

namespace skyvolt_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct AssignJob_Response_
{
  using Type = AssignJob_Response_<ContainerAllocator>;

  explicit AssignJob_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
      this->assigned_robot_id = "";
      this->reason = "";
    }
  }

  explicit AssignJob_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : assigned_robot_id(_alloc),
    reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
      this->assigned_robot_id = "";
      this->reason = "";
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _assigned_robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _assigned_robot_id_type assigned_robot_id;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__assigned_robot_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->assigned_robot_id = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reason = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    skyvolt_msgs::srv::AssignJob_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const skyvolt_msgs::srv::AssignJob_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<skyvolt_msgs::srv::AssignJob_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<skyvolt_msgs::srv::AssignJob_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      skyvolt_msgs::srv::AssignJob_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<skyvolt_msgs::srv::AssignJob_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      skyvolt_msgs::srv::AssignJob_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<skyvolt_msgs::srv::AssignJob_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<skyvolt_msgs::srv::AssignJob_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<skyvolt_msgs::srv::AssignJob_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__skyvolt_msgs__srv__AssignJob_Response
    std::shared_ptr<skyvolt_msgs::srv::AssignJob_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__skyvolt_msgs__srv__AssignJob_Response
    std::shared_ptr<skyvolt_msgs::srv::AssignJob_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AssignJob_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->assigned_robot_id != other.assigned_robot_id) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    return true;
  }
  bool operator!=(const AssignJob_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AssignJob_Response_

// alias to use template instance with default allocator
using AssignJob_Response =
  skyvolt_msgs::srv::AssignJob_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace skyvolt_msgs

namespace skyvolt_msgs
{

namespace srv
{

struct AssignJob
{
  using Request = skyvolt_msgs::srv::AssignJob_Request;
  using Response = skyvolt_msgs::srv::AssignJob_Response;
};

}  // namespace srv

}  // namespace skyvolt_msgs

#endif  // SKYVOLT_MSGS__SRV__DETAIL__ASSIGN_JOB__STRUCT_HPP_
