/// Node class declaration.
/// \author Mattis Kieffer (kieffer@igmr.rwth-aachen.de)
///
/// \copyright Copyright 2025 Institut für Getriebetechnik, Maschinendynamik und
/// Robotik, RWTH-Aachen University. All rights reserved.
///
/// TODO(kieffer): rename file
/// \license SPDX-License-Identifier: LGPL-3.0-or-later (see LICENSE at repo root)
#ifndef SCANCONTROL_LLS_DRIVER__LLS_DRIVER_HPP_
#define SCANCONTROL_LLS_DRIVER__LLS_DRIVER_HPP_
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "scancontrol_lls_driver/profile_publisher.hpp"
#include "scancontrol_lls_msgs/srv/discovery.hpp"
#include "scancontrol_lls_msgs/srv/set_interface.hpp"
#include "scanner_interface.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

// forward declaration
class CInterfaceLLT;
namespace scancontrol_lls
{

class LaserLineScanner : public rclcpp::Node
{
public:
  LaserLineScanner();

private:
  std::unique_ptr<ProfilePublisher> m_converter;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_dataPublisher;
  rclcpp::Service<scancontrol_lls_msgs::srv::Discovery>::SharedPtr m_discoveryService;
  rclcpp::Service<scancontrol_lls_msgs::srv::SetInterface>::SharedPtr m_setInterfaceService;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_startScanService;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_stopScanService;
  std::shared_ptr<CInterfaceLLT> m_sensor;
  std::string m_deviceInterface;
  std::string m_configFilePath;
  /// true while a scan is running; guards against reconfiguring or starting twice.
  std::atomic_bool m_scanning{false};

  static void Callback(const void * pucData, size_t uiSize, gpointer user_data);

  void setProfilePeriod(std::chrono::microseconds period, std::chrono::microseconds exposure);
  void setSensitivity(Sensitivity sensitivity);
  void setTrigger(Trigger trigger);
  /// Configure the sensor's width/intensity peak filter. Each bound is written
  /// into a 16-bit field, so values must lie in [0, 65535] and the minima must
  /// not exceed the maxima; violations throw ScannerError instead of silently
  /// truncating.
  void setPeakFilter(
    int64_t minWidth, int64_t maxWidth, int64_t minIntensity, int64_t maxIntensity);
  bool is30xxSeries();
  void startScan();
  void stopScan();

  static std::vector<std::string> GetInterfaces();
  void initializeInterface(const std::string & interface);

  void discoveryHandler(std::shared_ptr<scancontrol_lls_msgs::srv::Discovery::Response> response);
  void setInterfaceHandler(
    const std::shared_ptr<scancontrol_lls_msgs::srv::SetInterface::Request> request,
    std::shared_ptr<scancontrol_lls_msgs::srv::SetInterface::Response> response);
  void startScanHandler(std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void stopScanHandler(std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};
}   // namespace scancontrol_lls
#endif  // SCANCONTROL_LLS_DRIVER__LLS_DRIVER_HPP_
