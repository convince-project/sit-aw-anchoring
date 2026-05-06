// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ANCHORING_PROCESS_DEF__NODE_UTILS_HPP_
#define ANCHORING_PROCESS_DEF__NODE_UTILS_HPP_

#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"

namespace anchoring_process_def
{

/**
 * @brief Declares static ROS2 parameter and sets it to a given value
 * if it was not already declared.
 *
 * @param[in] node A node in which given parameter to be declared
 * @param[in] param_name The name of parameter
 * @param[in] default_value Parameter value to initialize with
 * @param[in] parameter_descriptor Parameter descriptor (optional)
 */
void declare_parameter_if_not_declared(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  const std::string & param_name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
  rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, default_value, parameter_descriptor);
  }
}

/**
 * @brief Declares static ROS2 parameter with given type if it was not already declared.
 *
 * @param[in] node A node in which given parameter to be declared
 * @param[in] param_type The type of parameter
 * @param[in] default_value Parameter value to initialize with
 * @param[in] parameter_descriptor Parameter descriptor (optional)
 */
void declare_parameter_if_not_declared(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  const std::string & param_name,
  const rclcpp::ParameterType & param_type,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
  rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, param_type, parameter_descriptor);
  }
}

/**
 * @brief Gets the type of plugin for the selected node and its plugin.
 * Actually seeks for the value of "<plugin_name>.plugin" parameter.
 *
 * @param[in] node Selected node
 * @param[in] plugin_name The name of plugin the type of which is being searched for
 * @return A string containing the type of plugin (the value of "<plugin_name>.plugin" parameter)
 */
std::string get_plugin_type_param(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  const std::string & plugin_name)
{
  declare_parameter_if_not_declared(node, plugin_name + ".plugin", rclcpp::PARAMETER_STRING);
  std::string plugin_type;
  try {
    if (!node->get_parameter(plugin_name + ".plugin", plugin_type)) {
      RCLCPP_FATAL(
        node->get_logger(), "Can not get 'plugin' param value for %s", plugin_name.c_str());
      exit(-1);
    }
  } catch (rclcpp::exceptions::ParameterUninitializedException & ex) {
    RCLCPP_FATAL(node->get_logger(), "'plugin' param not defined for %s", plugin_name.c_str());
    exit(-1);
  }

  return plugin_type;
}

/**
 * @brief Gets the database name of a knowledge domain for the selected node and its domain.
 * Actually seeks for the value of "<domain_name>.database_name" parameter.
 *
 * @param[in] node Selected node
 * @param[in] domain_name The name of knowledge domain the database name of which is being searched for
 * @return A strings containing the database name (the value of "<domain_name>.database_name" parameter)
 */
std::string get_domain_database_name_param(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  const std::string & domain_name)
{
  declare_parameter_if_not_declared(node, domain_name + ".database_name", rclcpp::PARAMETER_STRING);
  std::string database_name;
  try {
    if (!node->get_parameter(domain_name + ".database_name", database_name)) {
      RCLCPP_FATAL(
        node->get_logger(), "Can not get 'database_name' param value for domain %s", domain_name.c_str());
      exit(-1);
    }
  } catch (rclcpp::exceptions::ParameterUninitializedException & ex) {
    RCLCPP_FATAL(
      node->get_logger(), "'database_name' param not defined for domain %s", domain_name.c_str());
    exit(-1);
  }

  return database_name;
}

/**
 * @brief Gets the package name of a knowledge domain for the selected node and its domain.
 * Actually seeks for the value of "<domain_name>.pkg" parameter.
 *
 * @param[in] node Selected node
 * @param[in] domain_name The name of knowledge domain the package name of which is being searched for
 * @return A strings containing the package name (the value of "<domain_name>.pkg" parameter)
 */
std::string get_domain_pkg_param(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  const std::string & domain_name)
{
  declare_parameter_if_not_declared(node, domain_name + ".pkg", rclcpp::PARAMETER_STRING);
  std::string pkg;
  try {
    if (!node->get_parameter(domain_name + ".pkg", pkg)) {
      RCLCPP_FATAL(
        node->get_logger(), "Can not get 'pkg' param value for domain %s", domain_name.c_str());
      exit(-1);
    }
  } catch (rclcpp::exceptions::ParameterUninitializedException & ex) {
    RCLCPP_FATAL(
      node->get_logger(), "'pkg' param not defined for domain %s", domain_name.c_str());
    exit(-1);
  }

  return pkg;
}

/**
 * @brief Gets the schemas of a knowledge domain for the selected node and its domain.
 * Actually seeks for the value of "<domain_name>.schemas" parameter.
 *
 * @param[in] node Selected node
 * @param[in] domain_name The name of knowledge domain the schemas of which are being searched for
 * @return A vector of strings containing the schemas (the value of "<domain_name>.schemas" parameter)
 */
std::vector<std::string> get_domain_schemas_param(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  const std::string & domain_name)
{
  declare_parameter_if_not_declared(node, domain_name + ".schemas", rclcpp::PARAMETER_STRING_ARRAY);
  std::vector<std::string> schemas;
  try {
    if (!node->get_parameter(domain_name + ".schemas", schemas)) {
      RCLCPP_FATAL(
        node->get_logger(), "Can not get 'schemas' param value for domain %s", domain_name.c_str());
      exit(-1);
    }
  } catch (rclcpp::exceptions::ParameterUninitializedException & ex) {
    RCLCPP_FATAL(
      node->get_logger(), "'schemas' param not defined for domain %s", domain_name.c_str());
    exit(-1);
  }

  return schemas;
}

}  // namespace anchoring_process_def

#endif  // ANCHORING_PROCESS_DEF__NODE_UTILS_HPP_

