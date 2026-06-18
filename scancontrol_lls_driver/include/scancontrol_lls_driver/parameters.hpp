/// ROS2 Parameter declarations for the scancontrol_llt node.
/// \author Mattis Kieffer (kieffer@igmr.rwth-aachen.de)
///
/// \copyright Copyright 2025 Institut für Getriebetechnik, Maschinendynamik und
/// Robotik, RWTH-Aachen University. All rights reserved.
///
/// \license SPDX-License-Identifier: LGPL-3.0-or-later (see LICENSE at repo root)
#ifndef SCANCONTROL_LLS_DRIVER__PARAMETERS_HPP_
#define SCANCONTROL_LLS_DRIVER__PARAMETERS_HPP_

#include <array>
#include <optional>
#include <string_view>
#include <type_traits>

#include "parameter_helper.hpp"
#include "rclcpp/rclcpp.hpp"

namespace scancontrol_lls
{
namespace constants
{
/// @brief maximum number of bytes within a raw frame per scan point.
static constexpr std::size_t BYTES_PER_POINT{64};

/// @brief  milli prefix factor (1e-3)
static constexpr double MILLI{1e-3};
}  // namespace constants
namespace params
{

static constexpr ParameterDescriptor<std::string_view> DataTopic{
  "data_topic", "scancontrol_lls/scanPoints", "name for the sensor data (pointcloud) topic."};
static constexpr ParameterDescriptor<std::string_view> DiscoveryServiceName{
  "discovery_service", "scancontrol_lls/discoverDevices",
  "name for the service where the available interfaces will be provided."};
static constexpr ParameterDescriptor<std::string_view> InterfaceSelectionService{
  "interface_service", "scancontrol_lls/setInterface",
  "name for the service, where the desired "
  "interface name can be changed."};
static constexpr ParameterDescriptor<std::string_view> StartScanService{
  "start_scan", "scancontrol_lls/startScan", "Service name to start a scan."};
static constexpr ParameterDescriptor<std::string_view> StopScanService{
  "stop_scan", "scancontrol_lls/stopScan", "Service name to stop a scan."};

// Exposure and idle (= profile_period - exposure_time) are each encoded on the
// 30xx series in a 12-bit field of 10 us steps plus a 0..9 us offset, so each is
// capped at 4095 * 10 + 9 = 40959 us. exposure_time must always be < profile_period.
static constexpr ParameterDescriptor<int> ExposureTimeMicroseconds{
  "exposure_time", 1'000,
  "Exposure time for a single measurement in microseconds. Must be smaller than "
  "profile_period; capped at 40959 us on 30xx-series sensors.",
  std::array<int, 2>{1, 40'959}};

static constexpr ParameterDescriptor<int> ProfilePeriodMicroseconds{
  "profile_period", 10'000,
  "Time between two consecutive measurements in microseconds (idle = profile_period "
  "- exposure_time). On 30xx-series sensors idle is capped at 40959 us, so the "
  "period must not exceed exposure_time + 40959 us (<= 81918 us total).",
  std::array<int, 2>{2, 81'918}};

static constexpr ParameterDescriptor<int> FrameBufferSize{
  "frame_buffer_size", 256, "Buffer size for the rx buffer.", std::array<int, 2>{1, 2048}};

static constexpr ParameterDescriptor<std::string_view> ConfigFilePath{
  "configuration_file_path", "",
  "path to a complete configuration file to be read by the sensor sdk."};

static constexpr ParameterDescriptor<std::string_view> Sensitivity{
  "sensor_sensitivity", "Medium",
  "Sensitivity selection for the sensor. allwoed values: [Minimum, Low, "
  "Medium, High, Maximum]."};
static constexpr ParameterDescriptor<std::string_view> TriggerType{
  "sensor_trigger_source", "FreeRun",
  "Trigger source selection for the sensor. allwoed values: [SoftwareSingle, "
  "SoftwareFrame, HwSingle, HwEncoder, HwFrame, FreeRun]."};
static constexpr ParameterDescriptor<std::string_view> FrameName{
  "frame_name", "scanner_tf", "Frame name for the published pointcloud."};

static constexpr ParameterDescriptor<bool> ConnectOnStartup{
  "connect_on_startup", true,
  "If true, the node connects to the first available interface on startup. "
  "Connection failures are non-fatal; the SetInterface service can be used later."};

}  // namespace params
}  // namespace scancontrol_lls

#endif  // SCANCONTROL_LLS_DRIVER__PARAMETERS_HPP_
