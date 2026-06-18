/// MessagePoint header file.
/// \author Mattis Kieffer (kieffer@igmr.rwth-aachen.de)
///
/// \copyright Copyright 2025 Institut für Getriebetechnik, Maschinendynamik und
/// Robotik, RWTH-Aachen University. All rights reserved.
///
/// \license SPDX-License-Identifier: LGPL-3.0-or-later (see LICENSE at repo root)
#ifndef SCANCONTROL_LLS_DRIVER__MESSAGE_POINT_HPP_
#define SCANCONTROL_LLS_DRIVER__MESSAGE_POINT_HPP_
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace scancontrol_lls
{
//
struct __attribute__((packed)) MessagePoint
{
  float x{};
  float y{};
  float z{};
  uint32_t m0{};
  uint32_t m1{};
  uint16_t width{};
  uint16_t intensity{};
  uint16_t threshold{};

  static std::vector<sensor_msgs::msg::PointField> GetFieldDef();
  static sensor_msgs::msg::PointCloud2 NewPointCloud(size_t size);

  static MessagePoint * NewInPlace(sensor_msgs::msg::PointCloud2 & pc, size_t idx);
};

}  // namespace scancontrol_lls

#endif  // SCANCONTROL_LLS_DRIVER__MESSAGE_POINT_HPP_
