/// Profile publisher implementation file.
/// \author Mattis Kieffer (kieffer@igmr.rwth-aachen.de)
///
/// \copyright Copyright 2025 Institut für Getriebetechnik, Maschinendynamik und
/// Robotik, RWTH-Aachen University. All rights reserved.
///
/// \license SPDX-License-Identifier: LGPL-3.0-or-later (see LICENSE at repo root)
#include "scancontrol_lls_driver/profile_publisher.hpp"

namespace scancontrol_lls
{

ProfilePublisher::ProfilePublisher(
  size_t resolution, TScannerType type,
  std::function<void(std::unique_ptr<sensor_msgs::msg::PointCloud2>)>
  publisher,
  const std::string & frameName, size_t buffer_size)
: m_resolution(resolution),
  m_lltType(type),
  m_frameName(frameName),
  m_publisher(publisher),
  m_rxBuffers(constants::BYTES_PER_POINT * resolution, buffer_size)
{
  m_processorThread = std::thread(&ProfilePublisher::run, this);
}
ProfilePublisher::~ProfilePublisher() noexcept
{
  m_isRunning = false;
  m_rxBuffers.shutdown();
  if (m_processorThread.joinable()) {
    m_processorThread.join();
  }
}
// Copy the raw frame into a pooled buffer and hand it to the consumer thread.
void ProfilePublisher::emplaceData(
  const unsigned char * data, size_t size,
  rclcpp::Time time)
{
  auto frame = m_rxBuffers.newFrame(size, time);
  memcpy(frame.data().data(), data, size);
  m_rxBuffers.push_frame(std::move(frame));
}

void ProfilePublisher::run()
{
  std::vector<double> xValues(m_resolution);
  std::vector<double> zValues(m_resolution);
  std::vector<uint32_t> m0Values(m_resolution);
  std::vector<uint32_t> m1Values(m_resolution);
  std::vector<uint16_t> widthValues(m_resolution);
  std::vector<uint16_t> intensityValues(m_resolution);
  std::vector<uint16_t> thresholdValues(m_resolution);

  // ConvertProfile2Values returns a bitmask of the conversions it performed
  // (or a negative error code). X and Z are the geometry we publish, so require
  // both; anything else means the frame is unusable and is dropped.
  constexpr gint32 REQUIRED_CONVERSIONS = CONVERT_X | CONVERT_Z;
  const auto logger = rclcpp::get_logger("scancontrol_lls.profile_publisher");
  rclcpp::Clock throttle_clock{RCL_STEADY_TIME};

  while (m_isRunning) {
    auto rx_buffer = m_rxBuffers.pop_frame();
    if (m_isRunning && rx_buffer.has_value()) {  // process data
      // Convert raw data to mm
      const gint32 converted = CInterfaceLLT::ConvertProfile2Values(
        rx_buffer->data().data(), rx_buffer->data().size(), m_resolution,
        PROFILE, m_lltType, 0, widthValues.data(), intensityValues.data(),
        thresholdValues.data(), xValues.data(), zValues.data(),
        m0Values.data(), m1Values.data());
      if (converted < 0 || (converted & REQUIRED_CONVERSIONS) != REQUIRED_CONVERSIONS) {
        RCLCPP_WARN_THROTTLE(
          logger, throttle_clock, 1000,
          "Dropping frame: ConvertProfile2Values returned 0x%x (need X and Z)", converted);
        continue;
      }
      auto pc = std::make_unique<sensor_msgs::msg::PointCloud2>(
        MessagePoint::NewPointCloud(m_resolution));
      pc->header.stamp = rx_buffer->rxTime();
      pc->header.frame_id = m_frameName;
      for (size_t idx = 0; idx < m_resolution; ++idx) {
        auto * pt = MessagePoint::NewInPlace(*pc, idx);
        pt->x = xValues[idx] * constants::MILLI;
        pt->z = zValues[idx] * constants::MILLI;
        pt->m0 = m0Values[idx];
        pt->m1 = m1Values[idx];
        pt->intensity = intensityValues[idx];
        pt->width = widthValues[idx];
        pt->threshold = thresholdValues[idx];
      }
      m_publisher(std::move(pc));
    }
  }
}

}  // namespace scancontrol_lls
