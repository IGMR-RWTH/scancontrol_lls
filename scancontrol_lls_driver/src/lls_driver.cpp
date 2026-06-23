/// Node class implementation.
/// \author Mattis Kieffer (kieffer@igmr.rwth-aachen.de)
///
/// \copyright Copyright 2025 Institut für Getriebetechnik, Maschinendynamik und
/// Robotik, RWTH-Aachen University. All rights reserved.
///
/// TODO(kieffer): rename file
/// \license SPDX-License-Identifier: LGPL-3.0-or-later (see LICENSE at repo root)

#include "scancontrol_lls_driver/lls_driver.hpp"

#include <memory>
#include <memory_resource>
#include <string>

#include "llt.h"  // NOLINT
#include "rclcpp/rclcpp.hpp"
#include "scancontrol_lls_driver/parameters.hpp"
#include "scancontrol_lls_driver/profile_publisher.hpp"

namespace scancontrol_lls
{

LaserLineScanner::LaserLineScanner()
: Node("lls_driver"), m_sensor(std::make_shared<CInterfaceLLT>())
{
  using namespace std::placeholders;  // NOLINT
  using namespace params;             // NOLINT
  // register the nodes parameters.
  DataTopic.registerParam(this);
  DiscoveryServiceName.registerParam(this);
  InterfaceSelectionService.registerParam(this);
  ExposureTimeMicroseconds.registerParam(this);
  ProfilePeriodMicroseconds.registerParam(this);
  ConfigFilePath.registerParam(this);
  params::Sensitivity.registerParam(this);
  TriggerType.registerParam(this);
  FrameName.registerParam(this);
  StartScanService.registerParam(this);
  StopScanService.registerParam(this);
  FrameBufferSize.registerParam(this);
  ConnectOnStartup.registerParam(this);
  PeakFilterEnable.registerParam(this);
  PeakFilterMinWidth.registerParam(this);
  PeakFilterMaxWidth.registerParam(this);
  PeakFilterMinIntensity.registerParam(this);
  PeakFilterMaxIntensity.registerParam(this);
  RCLCPP_INFO(
    this->get_logger(), "Discovery service at: %s", DiscoveryServiceName.getRTValue(this).c_str());
  m_discoveryService = this->create_service<scancontrol_lls_msgs::srv::Discovery>(
    DiscoveryServiceName.getRTValue(this),
    std::bind(&LaserLineScanner::discoveryHandler, this, _2));
  RCLCPP_INFO(
    this->get_logger(), "Initialization service at: %s",
    InterfaceSelectionService.getRTValue(this).c_str());
  m_setInterfaceService = this->create_service<scancontrol_lls_msgs::srv::SetInterface>(
    InterfaceSelectionService.getRTValue(this),
    std::bind(&LaserLineScanner::setInterfaceHandler, this, _1, _2));
  RCLCPP_INFO(
    this->get_logger(), "StartScan service at: %s", StartScanService.getRTValue(this).c_str());
  m_startScanService = this->create_service<std_srvs::srv::Trigger>(
    StartScanService.getRTValue(this), std::bind(&LaserLineScanner::startScanHandler, this, _2));

  RCLCPP_INFO(
    this->get_logger(), "StopScan service at: %s", StopScanService.getRTValue(this).c_str());
  m_stopScanService = this->create_service<std_srvs::srv::Trigger>(
    StopScanService.getRTValue(this), std::bind(&LaserLineScanner::stopScanHandler, this, _2));
  RCLCPP_INFO(this->get_logger(), "publishing data to: %s", DataTopic.getRTValue(this).c_str());
  m_dataPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    DataTopic.getRTValue(this), rclcpp::SensorDataQoS());

  // Optionally connect to the first available interface on startup. Failures are
  // non-fatal: the node stays up so the SetInterface service can be used later.
  if (ConnectOnStartup.getRTValue(this)) {
    try {
      initializeInterface("");
    } catch (const ScannerError & e) {
      RCLCPP_WARN(
        this->get_logger(),
        "connect_on_startup: could not connect to a sensor (%s). Use the '%s' service to connect.",
        e.what(), InterfaceSelectionService.getRTValue(this).c_str());
    }
  }
}

bool LaserLineScanner::is30xxSeries()
{
  TScannerType type;
  ScannerError::Check(m_sensor->GetLLTType(&type), "failed to retrieve scanner type");
  // 30xx series
  return TScannerType::scanCONTROL30xx_25 <= type && type <= TScannerType::scanCONTROL30xx_xxx;
}

std::vector<std::string> LaserLineScanner::GetInterfaces()
{
  std::vector<char *> interfaces(5, nullptr);
  int if_count = CInterfaceLLT::GetDeviceInterfaces(interfaces.data(), interfaces.size());
  if (if_count < 0) {
    throw ScannerError("DiscoveryService: Failed to discover interfaces.", if_count);
  }
  std::vector<std::string> str_interfaces(if_count);
  for (int idx = 0; idx < if_count; ++idx) {
    if (interfaces[idx] != nullptr) {
      str_interfaces[idx] = std::string(interfaces[idx]);
    }
  }
  return str_interfaces;
}

void LaserLineScanner::initializeInterface(const std::string & interface)
{
  // copy to owning variable to avoid dangling references
  std::string cfgPath = params::ConfigFilePath.getRTValue(this);
  if (!cfgPath.empty()) {
    m_configFilePath = std::filesystem::path(cfgPath).string();
    ScannerError::Check(
      m_sensor->SetPathDeviceProperties(m_configFilePath.c_str()),
      "failed to set path for the configuration file: " + cfgPath);
  }

  if (interface.empty()) {
    // if no interface is selected, connect to the first one
    auto interfaces = LaserLineScanner::GetInterfaces();
    if (interfaces.empty()) {
      throw ScannerError("no scanner interfaces were discovered to connect to", 0);
    }
    // copy to owning variable to avoid dangling references
    m_deviceInterface = interfaces.front();
  } else {
    // copy to owning variable to avoid dangling references
    m_deviceInterface = interface;
  }
  ScannerError::Check(
    m_sensor->SetDeviceInterface(m_deviceInterface.c_str()),
    "failed to select the desired interface!");
  // Connect sensor
  ScannerError::Check(m_sensor->Connect(), "Failed to connect to sensor");
  RCLCPP_INFO(this->get_logger(), "Connected to sensor: '%s'", m_deviceInterface.c_str());
  setProfilePeriod(
    std::chrono::microseconds(params::ProfilePeriodMicroseconds.getRTValue(this)),
    std::chrono::microseconds(params::ExposureTimeMicroseconds.getRTValue(this)));

  setTrigger(TriggerFromString(params::TriggerType.getRTValue(this)));
  setSensitivity(SensitivityFromString(params::Sensitivity.getRTValue(this)));

  if (params::PeakFilterEnable.getRTValue(this)) {
    setPeakFilter(
      params::PeakFilterMinWidth.getRTValue(this),
      params::PeakFilterMaxWidth.getRTValue(this),
      params::PeakFilterMinIntensity.getRTValue(this),
      params::PeakFilterMaxIntensity.getRTValue(this));
  }
}

void LaserLineScanner::discoveryHandler(
  std::shared_ptr<scancontrol_lls_msgs::srv::Discovery::Response> response)
{
  try {
    response->success = true;
    response->devices = LaserLineScanner::GetInterfaces();
    for (auto && iface : response->devices) {
      RCLCPP_INFO(this->get_logger(), "found interface: %s", iface.c_str());
    }
  } catch (const ScannerError & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    response->success = false;
    response->errormessage = std::string(e.what());
  }
}

void LaserLineScanner::setInterfaceHandler(
  const std::shared_ptr<scancontrol_lls_msgs::srv::SetInterface::Request> request,
  std::shared_ptr<scancontrol_lls_msgs::srv::SetInterface::Response> response)
{
  if (m_scanning) {
    RCLCPP_WARN(this->get_logger(), "SetInterface rejected: a scan is currently running");
    response->success = false;
    response->errormessage = "cannot change interface while scanning; call stop_scan first";
    return;
  }
  try {
    initializeInterface(request->interface);
    response->success = true;
  } catch (const ScannerError & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to serve SetInterface request: '%s'. Error: '%s'",
      request->interface.c_str(), e.what());
    response->errormessage = e.what();
    response->success = false;
  }
}

void LaserLineScanner::startScanHandler(std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  try {
    startScan();
    response->success = true;
  } catch (const ScannerError & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to serve StartScan request: '%s'", e.what());
    response->message = e.what();
    response->success = false;
  }
}

void LaserLineScanner::stopScanHandler(std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  try {
    stopScan();
    response->success = true;
  } catch (const ScannerError & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to serve StopScan request: '%s'", e.what());
    response->message = e.what();
    response->success = false;
  }
}

void LaserLineScanner::setProfilePeriod(
  std::chrono::microseconds period, std::chrono::microseconds exposure)
{
  if (exposure >= period) {
    throw ScannerError(
            "Setting the exposure time greater than the profile period is not "
            "possible!",
            0);
  }

  if (is30xxSeries()) {
    size_t exposureTime = exposure.count();
    size_t idleTime = (period - exposure).count();
    // The 30xx register encodes time/10 in a 12-bit field plus a remainder
    // nibble, so the largest representable value is 4095*10 + 9 = 40959 us.
    // Reject larger values instead of silently truncating them with & 0xFFF.
    constexpr size_t MAX_30XX_TIME_US = 40959;
    if (exposureTime > MAX_30XX_TIME_US || idleTime > MAX_30XX_TIME_US) {
      throw ScannerError(
              "exposure/idle time exceeds the 30xx-encodable maximum of 40959 us", 0);
    }
    ScannerError::Check(
      m_sensor->SetFeature(
        FEATURE_FUNCTION_EXPOSURE_TIME,
        (((exposureTime % 10) << 12) & 0xF000) + ((exposureTime / 10) & 0xFFF)),
      "Failed to set exposure time");
    ScannerError::Check(
      m_sensor->SetFeature(
        FEATURE_FUNCTION_IDLE_TIME, (((idleTime % 10) << 12) & 0xF000) + ((idleTime / 10) & 0xFFF)),
      "Failed to set idle time");
  } else {  // all other types
    size_t exposureCount = exposure.count() / 10;
    size_t idleCount = period.count() / 10 - exposureCount;
    ScannerError::Check(
      m_sensor->SetFeature(FEATURE_FUNCTION_EXPOSURE_TIME, exposureCount),
      "Failed to set exposure time");
    ScannerError::Check(
      m_sensor->SetFeature(FEATURE_FUNCTION_IDLE_TIME, idleCount), "Failed to set idle time");
  }
}

void LaserLineScanner::setSensitivity(Sensitivity sensitivity)
{
  if (is30xxSeries()) {
    unsigned int imageFeatures = 0;
    unsigned int intSensitivity{0};
    switch (sensitivity) {
      case Sensitivity::Minimum:
        intSensitivity = SENSITIVITY_MINIMUM;
        break;
      case Sensitivity::Low:
        intSensitivity = SENSITIVITY_LOW;
        break;
      case Sensitivity::Medium:
        intSensitivity = SENSITIVITY_MEDIUM;
        break;
      case Sensitivity::High:
        intSensitivity = SENSITIVITY_HIGH;
        break;
      case Sensitivity::Maximum:
        intSensitivity = SENSITIVITY_MAXIMUM;
        break;

      default:
        break;
    }

    // Read-modify-write the image features register: clear the existing
    // sensitivity field (3 bits at offset 3) before applying the selected value,
    // otherwise sensitivity could only ever be raised, never lowered.
    ScannerError::Check(
      m_sensor->GetFeature(FEATURE_FUNCTION_IMAGE_FEATURES, &imageFeatures),
      "failed to read the image features register");
    imageFeatures &= ~(0x7u << 3);
    imageFeatures |= intSensitivity;
    ScannerError::Check(
      m_sensor->SetFeature(FEATURE_FUNCTION_IMAGE_FEATURES, imageFeatures),
      "failed to write the image features register");
  } else {
    RCLCPP_ERROR(this->get_logger(), "tried to set gain on a non 30xx series sensor");
  }
}

void LaserLineScanner::setPeakFilter(
  int64_t minWidth, int64_t maxWidth, int64_t minIntensity, int64_t maxIntensity)
{
  // The SDK writes each bound into a 16-bit field. Reject values that would not
  // survive the narrowing to gushort instead of silently wrapping them.
  auto checkRange = [](int64_t value, const char * name) {
    if (value < 0 || value > 65'535) {
      throw ScannerError(
              std::string("peak filter value out of range [0, 65535]: ") + name, 0);
    }
  };
  checkRange(minWidth, "peak_filter_min_width");
  checkRange(maxWidth, "peak_filter_max_width");
  checkRange(minIntensity, "peak_filter_min_intensity");
  checkRange(maxIntensity, "peak_filter_max_intensity");
  if (minWidth > maxWidth) {
    throw ScannerError("peak_filter_min_width must not exceed peak_filter_max_width", 0);
  }
  if (minIntensity > maxIntensity) {
    throw ScannerError(
            "peak_filter_min_intensity must not exceed peak_filter_max_intensity", 0);
  }

  ScannerError::Check(
    m_sensor->SetPeakFilter(
      static_cast<gushort>(minWidth), static_cast<gushort>(maxWidth),
      static_cast<gushort>(minIntensity), static_cast<gushort>(maxIntensity)),
    "failed to set peak filter");
  RCLCPP_INFO(
    this->get_logger(), "peak filter applied: width [%d, %d], intensity [%d, %d]",
    static_cast<int>(minWidth), static_cast<int>(maxWidth),
    static_cast<int>(minIntensity), static_cast<int>(maxIntensity));
}

void LaserLineScanner::setTrigger(Trigger trigger)
{
  switch (trigger) {
    // case Trigger::SoftwareSingle:
    // break;
    // case Trigger::SoftwareFrame:
    // break;
    // case Trigger::HwSingle:
    // break;
    // case Trigger::HwEncoder:
    // break;
    // case Trigger::HwFrame:
    // break;
    case Trigger::FreeRun:
      ScannerError::Check(
        m_sensor->SetFeature(FEATURE_FUNCTION_TRIGGER, TRIG_INTERNAL),
        "Failed to set FreeRun Trigger mode");
      break;
    default:
      throw ScannerError("TriggerMode not implemented yet!", 0);
      break;
  }
}

void LaserLineScanner::startScan()
{
  using namespace std::placeholders;
  if (m_scanning) {
    throw ScannerError("a scan is already running; call stop_scan first", 0);
  }
  guint32 res{0};
  TScannerType type{};
  m_sensor->GetResolution(&res);
  size_t resolution{res};
  m_sensor->GetLLTType(&type);
  m_converter = std::make_unique<decltype(m_converter)::element_type>(
    resolution, type, [pub = m_dataPublisher](std::unique_ptr<sensor_msgs::msg::PointCloud2> pc) {
      pub->publish(std::move(pc));
    }, params::FrameName.getRTValue(this),
    params::FrameBufferSize.getRTValue(this));
  ScannerError::Check(
    m_sensor->RegisterBufferCallback((gpointer) & LaserLineScanner::Callback, this),
    "failed to register buffer callback");
  ScannerError::Check(
    m_sensor->TransferProfiles(NORMAL_TRANSFER, true), "failed to start transfer");
  m_scanning = true;
}

void LaserLineScanner::stopScan()
{
  using namespace std::placeholders;
  if (!m_scanning) {
    throw ScannerError("no scan is currently running", 0);
  }
  ScannerError::Check(
    m_sensor->TransferProfiles(NORMAL_TRANSFER, false), "failed to disable profile transfer");
  ScannerError::Check(
    m_sensor->RegisterBufferCallback(nullptr, nullptr), "failed to unregister callback");
  m_scanning = false;
}
void LaserLineScanner::Callback(const void * data, size_t size, gpointer user_data)
{
  auto instance = reinterpret_cast<LaserLineScanner *>(user_data);
  instance->m_converter->emplaceData(
    reinterpret_cast<const unsigned char *>(data), size, instance->get_clock()->now());
}
}  // namespace scancontrol_lls
