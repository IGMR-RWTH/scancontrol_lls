/// Error and interface enum declarations
/// \author Mattis Kieffer (kieffer@igmr.rwth-aachen.de)
///
/// \copyright Copyright 2025 Institut für Getriebetechnik, Maschinendynamik und
/// Robotik, RWTH-Aachen University. All rights reserved.
///
/// TODO(kieffer): rename file
/// \license SPDX-License-Identifier: LGPL-3.0-or-later (see LICENSE at repo root)
#ifndef SCANCONTROL_LLS_DRIVER__SCANNER_INTERFACE_HPP_
#define SCANCONTROL_LLS_DRIVER__SCANNER_INTERFACE_HPP_
#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace scancontrol_lls
{

/**
 * @brief Exception class for handling errors from a C API.
 *
 * This class extends std::runtime_error to provide more specific information
 * about errors encountered when interfacing with a C API, including the
 * associated error code.
 */
class ScannerError : public std::runtime_error
{
public:
  /**
   * @brief Constructs a ScannerError exception.
   *
   * @param message The error message describing the issue.
   * @param errorCode The integer return code indicating the specific error.
   */
  ScannerError(const std::string & message, int errorCode);
  virtual ~ScannerError() = default;

  /**
   * @brief Gets the associated error code.
   *
   * @return The integer return code related to this exception.
   */
  int getErrorCode() const noexcept;
  /**
   * @brief Returns a formatted error message including the error code.
   *
   * Overrides std::runtime_error's what() method to append
   * the error code to the provided message.
   *
   * @return A string containing both the original message and
   *         the appended error code.
   */
  const char * what() const noexcept override;

  static void Check(int code, const std::string & msg = {});

private:
  int m_errorCode;  ///< The integer return code indicating the specific error
  std::string m_fullMessage;
};

enum class Sensitivity
{
  Minimum,
  Low,
  Medium,
  High,
  Maximum
};
Sensitivity SensitivityFromString(const std::string & str);

enum class Trigger
{
  SoftwareSingle,
  SoftwareFrame,
  HwSingle,
  HwEncoder,
  HwFrame,
  FreeRun
};
Trigger TriggerFromString(const std::string & str);
}  // namespace scancontrol_lls

#endif  // SCANCONTROL_LLS_DRIVER__SCANNER_INTERFACE_HPP_
