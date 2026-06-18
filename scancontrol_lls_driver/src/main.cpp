/// Node entrypoint.
/// \author Mattis Kieffer (kieffer@igmr.rwth-aachen.de)
///
/// \copyright Copyright 2025 Institut für Getriebetechnik, Maschinendynamik und
/// Robotik, RWTH-Aachen University. All rights reserved.
///
/// TODO(kieffer): rename file
/// \license SPDX-License-Identifier: LGPL-3.0-or-later (see LICENSE at repo root)

#include "scancontrol_lls_driver/lls_driver.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scancontrol_lls::LaserLineScanner>());
  rclcpp::shutdown();
  return 0;
}
