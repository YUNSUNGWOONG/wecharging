// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from skyvolt_msgs:msg/PhotoeyeDetection.idl
// generated code does not contain a copyright notice

#ifndef SKYVOLT_MSGS__MSG__DETAIL__PHOTOEYE_DETECTION__BUILDER_HPP_
#define SKYVOLT_MSGS__MSG__DETAIL__PHOTOEYE_DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "skyvolt_msgs/msg/detail/photoeye_detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace skyvolt_msgs
{

namespace msg
{

namespace builder
{

class Init_PhotoeyeDetection_docking_card_id
{
public:
  explicit Init_PhotoeyeDetection_docking_card_id(::skyvolt_msgs::msg::PhotoeyeDetection & msg)
  : msg_(msg)
  {}
  ::skyvolt_msgs::msg::PhotoeyeDetection docking_card_id(::skyvolt_msgs::msg::PhotoeyeDetection::_docking_card_id_type arg)
  {
    msg_.docking_card_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::skyvolt_msgs::msg::PhotoeyeDetection msg_;
};

class Init_PhotoeyeDetection_triggered
{
public:
  explicit Init_PhotoeyeDetection_triggered(::skyvolt_msgs::msg::PhotoeyeDetection & msg)
  : msg_(msg)
  {}
  Init_PhotoeyeDetection_docking_card_id triggered(::skyvolt_msgs::msg::PhotoeyeDetection::_triggered_type arg)
  {
    msg_.triggered = std::move(arg);
    return Init_PhotoeyeDetection_docking_card_id(msg_);
  }

private:
  ::skyvolt_msgs::msg::PhotoeyeDetection msg_;
};

class Init_PhotoeyeDetection_robot_id
{
public:
  explicit Init_PhotoeyeDetection_robot_id(::skyvolt_msgs::msg::PhotoeyeDetection & msg)
  : msg_(msg)
  {}
  Init_PhotoeyeDetection_triggered robot_id(::skyvolt_msgs::msg::PhotoeyeDetection::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_PhotoeyeDetection_triggered(msg_);
  }

private:
  ::skyvolt_msgs::msg::PhotoeyeDetection msg_;
};

class Init_PhotoeyeDetection_stamp
{
public:
  Init_PhotoeyeDetection_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PhotoeyeDetection_robot_id stamp(::skyvolt_msgs::msg::PhotoeyeDetection::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_PhotoeyeDetection_robot_id(msg_);
  }

private:
  ::skyvolt_msgs::msg::PhotoeyeDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::skyvolt_msgs::msg::PhotoeyeDetection>()
{
  return skyvolt_msgs::msg::builder::Init_PhotoeyeDetection_stamp();
}

}  // namespace skyvolt_msgs

#endif  // SKYVOLT_MSGS__MSG__DETAIL__PHOTOEYE_DETECTION__BUILDER_HPP_
