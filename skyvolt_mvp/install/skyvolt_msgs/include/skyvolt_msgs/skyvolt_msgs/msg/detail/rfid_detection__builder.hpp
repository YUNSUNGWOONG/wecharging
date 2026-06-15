// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from skyvolt_msgs:msg/RfidDetection.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__RFID_DETECTION__BUILDER_HPP_
#define SKYVOLT_MSGS__MSG__DETAIL__RFID_DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "skyvolt_msgs/msg/detail/rfid_detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace skyvolt_msgs
{

namespace msg
{

namespace builder
{

class Init_RfidDetection_rssi
{
public:
  explicit Init_RfidDetection_rssi(::skyvolt_msgs::msg::RfidDetection & msg)
  : msg_(msg)
  {}
  ::skyvolt_msgs::msg::RfidDetection rssi(::skyvolt_msgs::msg::RfidDetection::_rssi_type arg)
  {
    msg_.rssi = std::move(arg);
    return std::move(msg_);
  }

private:
  ::skyvolt_msgs::msg::RfidDetection msg_;
};

class Init_RfidDetection_expected_arclength_m
{
public:
  explicit Init_RfidDetection_expected_arclength_m(::skyvolt_msgs::msg::RfidDetection & msg)
  : msg_(msg)
  {}
  Init_RfidDetection_rssi expected_arclength_m(::skyvolt_msgs::msg::RfidDetection::_expected_arclength_m_type arg)
  {
    msg_.expected_arclength_m = std::move(arg);
    return Init_RfidDetection_rssi(msg_);
  }

private:
  ::skyvolt_msgs::msg::RfidDetection msg_;
};

class Init_RfidDetection_tag_id
{
public:
  explicit Init_RfidDetection_tag_id(::skyvolt_msgs::msg::RfidDetection & msg)
  : msg_(msg)
  {}
  Init_RfidDetection_expected_arclength_m tag_id(::skyvolt_msgs::msg::RfidDetection::_tag_id_type arg)
  {
    msg_.tag_id = std::move(arg);
    return Init_RfidDetection_expected_arclength_m(msg_);
  }

private:
  ::skyvolt_msgs::msg::RfidDetection msg_;
};

class Init_RfidDetection_kind
{
public:
  explicit Init_RfidDetection_kind(::skyvolt_msgs::msg::RfidDetection & msg)
  : msg_(msg)
  {}
  Init_RfidDetection_tag_id kind(::skyvolt_msgs::msg::RfidDetection::_kind_type arg)
  {
    msg_.kind = std::move(arg);
    return Init_RfidDetection_tag_id(msg_);
  }

private:
  ::skyvolt_msgs::msg::RfidDetection msg_;
};

class Init_RfidDetection_robot_id
{
public:
  explicit Init_RfidDetection_robot_id(::skyvolt_msgs::msg::RfidDetection & msg)
  : msg_(msg)
  {}
  Init_RfidDetection_kind robot_id(::skyvolt_msgs::msg::RfidDetection::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_RfidDetection_kind(msg_);
  }

private:
  ::skyvolt_msgs::msg::RfidDetection msg_;
};

class Init_RfidDetection_stamp
{
public:
  Init_RfidDetection_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RfidDetection_robot_id stamp(::skyvolt_msgs::msg::RfidDetection::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_RfidDetection_robot_id(msg_);
  }

private:
  ::skyvolt_msgs::msg::RfidDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::skyvolt_msgs::msg::RfidDetection>()
{
  return skyvolt_msgs::msg::builder::Init_RfidDetection_stamp();
}

}  // namespace skyvolt_msgs

#endif  // SKYVOLT_MSGS__MSG__DETAIL__RFID_DETECTION__BUILDER_HPP_
