// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from skyvolt_msgs:msg/PhotoeyeDetection.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__PHOTOEYE_DETECTION__STRUCT_HPP_
#define SKYVOLT_MSGS__MSG__DETAIL__PHOTOEYE_DETECTION__STRUCT_HPP_

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
# define DEPRECATED__skyvolt_msgs__msg__PhotoeyeDetection __attribute__((deprecated))
#else
# define DEPRECATED__skyvolt_msgs__msg__PhotoeyeDetection __declspec(deprecated)
#endif

namespace skyvolt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PhotoeyeDetection_
{
  using Type = PhotoeyeDetection_<ContainerAllocator>;

  explicit PhotoeyeDetection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->triggered = false;
      this->docking_card_id = "";
    }
  }

  explicit PhotoeyeDetection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    robot_id(_alloc),
    docking_card_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->triggered = false;
      this->docking_card_id = "";
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_id_type robot_id;
  using _triggered_type =
    bool;
  _triggered_type triggered;
  using _docking_card_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _docking_card_id_type docking_card_id;

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
  Type & set__triggered(
    const bool & _arg)
  {
    this->triggered = _arg;
    return *this;
  }
  Type & set__docking_card_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->docking_card_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    skyvolt_msgs::msg::PhotoeyeDetection_<ContainerAllocator> *;
  using ConstRawPtr =
    const skyvolt_msgs::msg::PhotoeyeDetection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<skyvolt_msgs::msg::PhotoeyeDetection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<skyvolt_msgs::msg::PhotoeyeDetection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      skyvolt_msgs::msg::PhotoeyeDetection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<skyvolt_msgs::msg::PhotoeyeDetection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      skyvolt_msgs::msg::PhotoeyeDetection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<skyvolt_msgs::msg::PhotoeyeDetection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<skyvolt_msgs::msg::PhotoeyeDetection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<skyvolt_msgs::msg::PhotoeyeDetection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__skyvolt_msgs__msg__PhotoeyeDetection
    std::shared_ptr<skyvolt_msgs::msg::PhotoeyeDetection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__skyvolt_msgs__msg__PhotoeyeDetection
    std::shared_ptr<skyvolt_msgs::msg::PhotoeyeDetection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PhotoeyeDetection_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->triggered != other.triggered) {
      return false;
    }
    if (this->docking_card_id != other.docking_card_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const PhotoeyeDetection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PhotoeyeDetection_

// alias to use template instance with default allocator
using PhotoeyeDetection =
  skyvolt_msgs::msg::PhotoeyeDetection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace skyvolt_msgs

#endif  // SKYVOLT_MSGS__MSG__DETAIL__PHOTOEYE_DETECTION__STRUCT_HPP_
