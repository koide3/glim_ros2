#pragma once

#include <functional>

#ifdef BUILD_WITH_CV_BRIDGE
#ifdef CV_BRIDGE_INCLUDE_H
// For ROS2 humble or earlier
#include <cv_bridge/cv_bridge.h>
#elif CV_BRIDGE_INCLUDE_HPP
// For ROS2 jazzy and later
#include <cv_bridge/cv_bridge.hpp>
#else
#error File extension of cv_bridge is unknown!!
#endif
#endif

// For ROS2 humble or earlier
template <typename Msg>
std::enable_if_t<sizeof(&Msg::time_stamp) != 0, int64_t> get_msg_recv_timestamp(const Msg& msg) {
  return msg.time_stamp;
}

// For ROS2 jazzy and later
template <typename Msg>
std::enable_if_t<sizeof(&Msg::recv_timestamp) != 0, int64_t> get_msg_recv_timestamp(const Msg& msg) {
  return msg.recv_timestamp;
}
