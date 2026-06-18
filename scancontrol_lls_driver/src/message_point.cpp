/// MessagePoint implementation file.
/// \author Mattis Kieffer (kieffer@igmr.rwth-aachen.de)
///
/// \copyright Copyright 2025 Institut für Getriebetechnik, Maschinendynamik und
/// Robotik, RWTH-Aachen University. All rights reserved.
///
/// \license SPDX-License-Identifier: LGPL-3.0-or-later (see LICENSE at repo root)

#include "scancontrol_lls_driver/message_point.hpp"

#include "scancontrol_lls_driver/scanner_interface.hpp"
namespace scancontrol_lls
{
std::vector<sensor_msgs::msg::PointField> MessagePoint::GetFieldDef()
{
  std::vector<sensor_msgs::msg::PointField> fields(8);
  fields[0].count = 1;
  fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[0].name = "x";  //< TODO(kieffer): make configurable
  fields[0].offset = 0;

  // needed for the rviz pointcloud vizualization plugin...It only accepts
  // float32 and expects x, y and z fields to be present
  fields[1].count = 1;
  fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[1].name = "y";  //< TODO(kieffer): make configurable
  fields[1].offset = 4;

  fields[2].count = 1;
  fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[2].name = "z";  //< TODO(kieffer): make configurable
  fields[2].offset = 8;

  fields[3].count = 1;
  fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
  fields[3].name = "m0";  //< TODO(kieffer): make configurable
  fields[3].offset = 12;

  fields[4].count = 1;
  fields[4].datatype = sensor_msgs::msg::PointField::UINT32;
  fields[4].name = "m1";  //< TODO(kieffer): make configurable
  fields[4].offset = 16;

  fields[5].count = 1;
  fields[5].datatype = sensor_msgs::msg::PointField::UINT16;
  fields[5].name = "width";  //< TODO(kieffer): make configurable
  fields[5].offset = 20;

  fields[6].count = 1;
  fields[6].datatype = sensor_msgs::msg::PointField::UINT16;
  fields[6].name = "intensity";  //< TODO(kieffer): make configurable
  fields[6].offset = 22;

  fields[7].count = 1;
  fields[7].datatype = sensor_msgs::msg::PointField::UINT16;
  fields[7].name = "threshold";  //< TODO(kieffer): make configurable
  fields[7].offset = 24;
  static_assert(sizeof(MessagePoint) == 26, "size to offset mismatch!");
  return fields;
}

sensor_msgs::msg::PointCloud2 MessagePoint::NewPointCloud(size_t size)
{
  auto points = sensor_msgs::msg::PointCloud2();
  points.header.frame_id = "map";
  points.fields = MessagePoint::GetFieldDef();
  points.data.resize(sizeof(MessagePoint) * size);
  points.height = 1;
  points.width = size;
  points.point_step = sizeof(MessagePoint);
  points.row_step = size * points.point_step;
  points.is_dense = true;
  return points;
}

MessagePoint * MessagePoint::NewInPlace(sensor_msgs::msg::PointCloud2 & pc, size_t idx)
{
  auto start_addr = pc.point_step * idx;
  if (pc.data.size() < start_addr + pc.point_step) {
    throw ScannerError("tried to allocate message point outside buffer!", 0);
  }
  return new (&pc.data[start_addr]) MessagePoint();
}

}  // namespace scancontrol_lls
