// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from skyvolt_msgs:msg/RfidDetection.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__RFID_DETECTION__STRUCT_HPP_
#define SKYVOLT_MSGS__MSG__DETAIL__RFID_DETECTION__STRUCT_HPP_

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
# define DEPRECATED__skyvolt_msgs__msg__RfidDetection __attribute__((deprecated))
#else
# define DEPRECATED__skyvolt_msgs__msg__RfidDetection __declspec(deprecated)
#endif

namespace skyvolt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RfidDetection_
{
  using Type = RfidDetection_<ContainerAllocator>;

  explicit RfidDetection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->kind = 0;
      this->tag_id = "";
      this->expected_arclength_m = 0.0;
      this->rssi = 0.0;
    }
  }

  explicit RfidDetection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    robot_id(_alloc),
    tag_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->kind = 0;
      this->tag_id = "";
      this->expected_arclength_m = 0.0;
      this->rssi = 0.0;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_id_type robot_id;
  using _kind_type =
    uint8_t;
  _kind_type kind;
  using _tag_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _tag_id_type tag_id;
  using _expected_arclength_m_type =
    double;
  _expected_arclength_m_type expected_arclength_m;
  using _rssi_type =
    double;
  _rssi_type rssi;

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
  Type & set__kind(
    const uint8_t & _arg)
  {
    this->kind = _arg;
    return *this;
  }
  Type & set__tag_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->tag_id = _arg;
    return *this;
  }
  Type & set__expected_arclength_m(
    const double & _arg)
  {
    this->expected_arclength_m = _arg;
    return *this;
  }
  Type & set__rssi(
    const double & _arg)
  {
    this->rssi = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t KIND_CUE =
    1u;
  static constexpr uint8_t KIND_POSITIONING =
    2u;
  static constexpr uint8_t KIND_DOCKING =
    3u;

  // pointer types
  using RawPtr =
    skyvolt_msgs::msg::RfidDetection_<ContainerAllocator> *;
  using ConstRawPtr =
    const skyvolt_msgs::msg::RfidDetection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<skyvolt_msgs::msg::RfidDetection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<skyvolt_msgs::msg::RfidDetection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      skyvolt_msgs::msg::RfidDetection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<skyvolt_msgs::msg::RfidDetection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      skyvolt_msgs::msg::RfidDetection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<skyvolt_msgs::msg::RfidDetection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<skyvolt_msgs::msg::RfidDetection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<skyvolt_msgs::msg::RfidDetection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__skyvolt_msgs__msg__RfidDetection
    std::shared_ptr<skyvolt_msgs::msg::RfidDetection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__skyvolt_msgs__msg__RfidDetection
    std::shared_ptr<skyvolt_msgs::msg::RfidDetection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RfidDetection_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->kind != other.kind) {
      return false;
    }
    if (this->tag_id != other.tag_id) {
      return false;
    }
    if (this->expected_arclength_m != other.expected_arclength_m) {
      return false;
    }
    if (this->rssi != other.rssi) {
      return false;
    }
    return true;
  }
  bool operator!=(const RfidDetection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RfidDetection_

// alias to use template instance with default allocator
using RfidDetection =
  skyvolt_msgs::msg::RfidDetection_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RfidDetection_<ContainerAllocator>::KIND_CUE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RfidDetection_<ContainerAllocator>::KIND_POSITIONING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RfidDetection_<ContainerAllocator>::KIND_DOCKING;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace skyvolt_msgs

#endif  // SKYVOLT_MSGS__MSG__DETAIL__RFID_DETECTION__STRUCT_HPP_
