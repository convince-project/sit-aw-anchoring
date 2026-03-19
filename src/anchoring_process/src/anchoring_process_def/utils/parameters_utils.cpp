// include associated header file
#include "anchoring_process_def/anchoring_process_impl.h"

#include "node_utils.hpp"

namespace anchoring_process_def {

/**
 *
 */
bool anchoring_process_impl::loadAnchoringPlugins()
{
  auto node = shared_from_this();

  managers_types_.resize(managers_ids_.size());

  for (size_t i = 0; i != managers_ids_.size(); i++) {
    try {
      managers_types_[i] = get_plugin_type_param(node, managers_ids_[i]);
      anchoring_core::AnchoringManager::Ptr manager =
            am_loader_.createUniqueInstance(managers_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created anchoring manager: %s of type %s",
        managers_ids_[i].c_str(), managers_types_[i].c_str());
      manager->configure(node); // managers_ids_[i] ?
      managers_.insert({managers_ids_[i], manager});
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create anchoring manager. Exception: %s", ex.what());
      return false;
    }
  }
  return true;
}

/**
 *
 */
bool anchoring_process_impl::findOntologySchemas()
{
  auto node = shared_from_this();

  for (size_t i = 0; i != domains_ids_.size(); i++) {
    try {
      auto pkg = get_domain_pkg_param(node, domains_ids_[i]);
      std::string pkg_share_dir = ament_index_cpp::get_package_share_directory(pkg);
      auto domain_schemas = get_domain_schemas_param(node, domains_ids_[i]);
      // adjust path to schemas and test accessibility
      std::vector<std::string> pkg_domain_schemas;
      for (std::string& s : domain_schemas) {
        std::string pkg_s = pkg_share_dir+"/"+s;
        if (!std::ifstream(pkg_s).good())
        {
          RCLCPP_FATAL(
            get_logger(), "Failed to access domain schema at %s", pkg_s.c_str());
          return false;
        }
        pkg_domain_schemas.push_back(pkg_s);
      }
      schemas_.insert({domains_ids_[i], pkg_domain_schemas});
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to access domain schemas. Exception: %s", ex.what());
      return false;
    }
  }
  return true;
}

/**
 *
 */
bool anchoring_process_impl::createOrReplaceDatabases()
{
  auto node = shared_from_this();

  for (size_t i = 0; i != domains_ids_.size(); i++) {
    try {
      auto domain_database_name = get_domain_database_name_param(node, domains_ids_[i]);
      bool ok = TypeDBClient::create_or_replace_database(driver_, domain_database_name, true); // dbReset:=true
      if (!ok)
      {
        RCLCPP_FATAL(
          get_logger(), "Failed to create or replace domain database %s", domain_database_name.c_str());
        return false;
      }
      databases_.insert({domains_ids_[i], domain_database_name});
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to access domain database name. Exception: %s", ex.what());
      return false;
    }
  }
  return true;
}

} // of namespace anchoring_process_def
