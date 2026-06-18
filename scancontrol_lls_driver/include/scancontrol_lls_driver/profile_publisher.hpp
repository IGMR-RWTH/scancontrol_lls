/// Profile publisher header file.
/// \author Mattis Kieffer (kieffer@igmr.rwth-aachen.de)
///
/// \copyright Copyright 2025 Institut für Getriebetechnik, Maschinendynamik und
/// Robotik, RWTH-Aachen University. All rights reserved.
///
/// \license SPDX-License-Identifier: LGPL-3.0-or-later (see LICENSE at repo root)
#ifndef SCANCONTROL_LLS_DRIVER__PROFILE_PUBLISHER_HPP_
#define SCANCONTROL_LLS_DRIVER__PROFILE_PUBLISHER_HPP_
#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "llt.h"  // NOLINT
#include "rclcpp/rclcpp.hpp"
#include "scancontrol_lls_driver/buffer.hpp"
#include "scancontrol_lls_driver/message_point.hpp"
#include "scancontrol_lls_driver/parameters.hpp"
#include "scancontrol_lls_driver/scanner_interface.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
namespace scancontrol_lls
{

class ProfilePublisher
{
public:
  ProfilePublisher(
    size_t resolution, TScannerType type,
    std::function<void(std::unique_ptr<sensor_msgs::msg::PointCloud2>)>
    publisher,
    const std::string & frameName, size_t buffer_size);

  ~ProfilePublisher() noexcept;
  // Enqueue a raw frame (copied into a pooled buffer) for the consumer thread.
  void emplaceData(const unsigned char * data, size_t size, rclcpp::Time time);

private:
  const size_t m_resolution;
  const TScannerType m_lltType;
  const std::string m_frameName;
  std::function<void(std::unique_ptr<sensor_msgs::msg::PointCloud2>)>
  m_publisher;
  std::atomic_bool m_isRunning{true};
  Buffer m_rxBuffers;
  // Single consumer thread: it pops raw frames from m_rxBuffers in FIFO order,
  // converts them and publishes. Using exactly one thread keeps the published
  // point clouds in acquisition order (multiple workers would reorder them).
  std::thread m_processorThread;
  // wait until a frame is available, move it out of the queue, convert and
  // publish it.
  void run();
};

}  // namespace scancontrol_lls

#endif  // SCANCONTROL_LLS_DRIVER__PROFILE_PUBLISHER_HPP_
