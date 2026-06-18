/// ROS2 Parameter declaration helper Header
/// \author Mattis Kieffer (kieffer@igmr.rwth-aachen.de)
///
/// \copyright Copyright 2025 Institut für Getriebetechnik, Maschinendynamik und
/// Robotik, RWTH-Aachen University. All rights reserved.
///
/// \license SPDX-License-Identifier: LGPL-3.0-or-later (see LICENSE at repo root)

#ifndef SCANCONTROL_LLS_DRIVER__PARAMETER_HELPER_HPP_
#define SCANCONTROL_LLS_DRIVER__PARAMETER_HELPER_HPP_

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"

namespace scancontrol_lls
{
namespace params
{

/**
 * @brief A helper class to define parameters for ROS2 nodes.
 *
 * This class allows the static constexpr declaration of parameters,
 * including their names, default values, limits, and descriptions.
 * This enables declaration of parameters inside a single header file to improve
 * code maintainability.
 *
 * @tparam T The type of the parameter.
 */
template<typename T>
class ParameterDescriptor
{
public:
  /**
   * @brief Constructs a ParameterDescriptor object.
   *
   * @param param_name The name of the parameter.
   * @param defaultValue The default value of the parameter.
   * @param limits NOT IMPLEMENTED! Optional array specifying min and max limits
   * for the parameter (if applicable).
   * @param description Optional string providing a description of the
   * parameter.
   */
  constexpr ParameterDescriptor(
    std::string_view param_name, T defaultValue,
    std::optional<std::string_view> description = std::nullopt,
    std::optional<std::array<T, 2>> limits = std::nullopt)
  : m_name(param_name), m_default(defaultValue), m_description(description), m_limits(limits)
  {
  }

  /**
   * @brief Gets the name of the declared parameter.
   *
   * @return A string representing the name of the parameter.
   */
  std::string getName() const {return std::string(m_name);}

  /**
   * @brief Gets the default value of the parameter.
   *
   * Converts strings to `std::string` if necessary for ROS interoperability
   *
   * @return The default value of type T or converted to `std::string`.
   */
  auto getDefault() const
  {
    if constexpr (
      std::is_same_v<std::decay_t<T>, std::string>||
      std::is_same_v<std::decay_t<T>, std::string_view>)
    {
      return std::string(m_default);
    } else {
      return m_default;
    }
  }

  /**
   * @brief Creates a ROS2 ParameterDescriptor message from this descriptor.
   *
   * Populates a rcl_interfaces.msg.ParameterDescriptor with relevant
   * information such as name and description.
   *
   * @return A populated rcl_interfaces.msg.ParameterDescriptor object.
   */
  rcl_interfaces::msg::ParameterDescriptor getDescriptor() const
  {
    auto desc = rcl_interfaces::msg::ParameterDescriptor{};
    desc.name = getName();

    if (m_description.has_value()) {
      desc.description = *m_description;
    }

    /// TODO: Add other descriptor fields like limits

    return desc;
  }

  /**
   * @brief Registers this parameter with a given ROS2 node.
   *
   * Declares the parameter in the provided node using its name, default value,
   * and descriptor information.
   *
   * @param node Pointer to an instance of rclcpp Node where this parameter
   * should be registered.
   */
  void registerParam(rclcpp::Node * node) const
  {
    node->declare_parameter(getName(), getDefault(), getDescriptor());
  }

  /**
   * @brief Retrieves the runtime value of this parameter from a given ROS2
   * node.
   *
   * Uses appropriate methods based on type T to retrieve values correctly.
   *
   * @param node Pointer to an instance of rclcpp Node from which to retrieve
   * the parameter's value.
   * @return The current runtime value of type T retrieved from the specified
   * node.
   */
  auto getRTValue(rclcpp::Node * node) const
  {
    static_assert(!std::is_void_v<T>, "Unsupported parameter type");
    if constexpr (
      std::is_same_v<std::decay_t<T>, std::string>||
      std::is_same_v<std::decay_t<T>, std::string_view>)
    {
      return node->get_parameter(getName()).template get_value<std::string>();
    } else if constexpr (std::is_same_v<std::decay_t<T>, double>) {
      return node->get_parameter(getName()).template get_value<double>();
    } else if constexpr (std::is_same_v<std::decay_t<T>, bool>) {
      return node->get_parameter(getName()).template get_value<bool>();
    } else if constexpr (std::is_integral_v<std::decay_t<T>>) {
      return node->get_parameter(getName()).template get_value<int64_t>();
    } else {
      return node->get_parameter(getName()).template get_value<T>();
    }
  }

private:
  std::string_view m_name;                          ///< Name of the parameter as a string literal
  T m_default;                                      ///< Default value for this parameter
  std::optional<std::string_view> m_description;    ///< Optional description for clarity
  std::optional<std::array<T, 2>> m_limits;         ///< Optional limits for numeric parameters
};

}  // namespace params
}  // namespace scancontrol_lls

#endif  // SCANCONTROL_LLS_DRIVER__PARAMETER_HELPER_HPP_
