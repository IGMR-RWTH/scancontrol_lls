/// Error and api type implementations
/// \author Mattis Kieffer (kieffer@igmr.rwth-aachen.de)
///
/// \copyright Copyright 2025 Institut für Getriebetechnik, Maschinendynamik und
/// Robotik, RWTH-Aachen University. All rights reserved.
///
/// TODO(kieffer): rename file
/// \license SPDX-License-Identifier: LGPL-3.0-or-later (see LICENSE at repo root)
#include "scancontrol_lls_driver/scanner_interface.hpp"

namespace scancontrol_lls
{

ScannerError::ScannerError(const std::string & message, int errorCode)
: std::runtime_error(message), m_errorCode(errorCode)
{
  // Create a formatted string with original message and error code
  std::ostringstream oss;

  oss << std::runtime_error::what() << " (LLT error code: " << m_errorCode << ")";

  // Store the formatted message in a member so what() can return a stable
  // pointer for the lifetime of this exception object.
  m_fullMessage = oss.str();
}

/**
   * @brief Gets the associated error code.
   *
   * @return The integer return code related to this exception.
   */
int ScannerError::getErrorCode() const noexcept {return m_errorCode;}
/**
   * @brief Returns a formatted error message including the error code.
   *
   * Overrides std::runtime_error's what() method to append
   * the error code to the provided message.
   *
   * @return A string containing both the original message and
   *         the appended error code.
   */
const char * ScannerError::what() const noexcept {return m_fullMessage.c_str();}

void ScannerError::Check(int code, const std::string & msg)
{
  if (code < 0) {
    throw ScannerError(msg, code);
  }
}

Sensitivity SensitivityFromString(const std::string & str)
{
  if (str == "Minimum") {
    return Sensitivity::Minimum;
  } else if (str == "Low") {
    return Sensitivity::Low;
  } else if (str == "Medium") {
    return Sensitivity::Medium;
  } else if (str == "High") {
    return Sensitivity::High;
  } else if (str == "Maximum") {
    return Sensitivity::Maximum;
  } else {
    throw ScannerError("failed to parse sensitivity", 0);
  }
}

Trigger TriggerFromString(const std::string & str)
{
  if (str == "SoftwareSingle") {
    return Trigger::SoftwareSingle;
  } else if (str == "SoftwareFrame") {
    return Trigger::SoftwareFrame;
  } else if (str == "HwSingle") {
    return Trigger::HwSingle;
  } else if (str == "HwEncoder") {
    return Trigger::HwEncoder;
  } else if (str == "HwFrame") {
    return Trigger::HwFrame;
  } else if (str == "FreeRun") {
    return Trigger::FreeRun;
  } else {
    throw ScannerError("failed to parse trigger", 0);
  }
}
}  // namespace scancontrol_lls
