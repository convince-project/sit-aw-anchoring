#include "ament_index_cpp/get_package_share_directory.hpp"

#include "anchoring_skrawl_plugins/physical_entity_manager.hpp"

#include <fstream>


namespace anchoring_skrawl_plugins
{

  using json = nlohmann::json;

  void PhysicalEntityManager::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent)
  {
      node_ = parent;
      auto node = parent.lock();
      logger_ = node->get_logger();
      RCLCPP_INFO(logger_, "Configuring plugin of type %s", getType().c_str());

      std::string pkg_share_dir =
        ament_index_cpp::get_package_share_directory("anchoring_skrawl_plugins");

      // Mapping rules
      std::ifstream fs(pkg_share_dir+"/skrawl/dt-mapping/rules_box_shape.json");
      json data = json::parse(fs);
      // - configure
      for (const auto &r : data)
      {
        mappings_box_shape_.push_back({ r.value("json-path",""), r.value("target-attr","") });
      }

      std::ifstream fp(pkg_share_dir+"/skrawl/dt-mapping/rules_position3d.json");
      data = json::parse(fp);
      // - configure
      for (const auto &r : data)
      {
        mappings_position3d_.push_back({ r.value("json-path",""), r.value("target-attr","") });
      }

      RCLCPP_INFO(logger_, "Configured plugin of type %s", getType().c_str());
  }

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  void PhysicalEntityManager::cleanup()
  {
      RCLCPP_INFO(logger_, "Cleaning up plugin of type %s", getType().c_str());
  }

  /**
   * @brief Method to active the manager and any threads involved in execution.
   */
  void PhysicalEntityManager::activate()
  {
      RCLCPP_INFO(logger_, "Activating plugin of type %s", getType().c_str());
  }

  /**
   * @brief Method to deactivate the manager and any threads involved in execution.
   */
  void PhysicalEntityManager::deactivate()
  {
      RCLCPP_INFO(logger_, "Deactivating plugin of type %s", getType().c_str());
  }

  /**
   * @brief ...
   */
  std::vector<std::string> PhysicalEntityManager::generatePopulateInstanceQueries(const json& elem)
  {
    // return value
    std::vector<std::string> queries;

    // add physical object-specific processing of data
    auto id = elem.value("id", "");

    // - box shape (with default values)
    std::string q = "match $a isa " + getType() + ", has id \""   + id     + "\"; "
                    "insert $bs isa box_shape, has height 0.0, has width 0.0, has length 0.0; "
                           "(owner: $a, shape: $bs) isa has_shape; ";
    queries.push_back(q);

    // - position3d (with default values)
    q = "match $a isa " + getType() + ", has id \""   + id     + "\"; "
        "insert $p isa position3d, has lin_x 0.0, has lin_y 0.0, has lin_z 0.0; "
               "(owner: $a, position: $p) isa has_position; ";
    queries.push_back(q);

    // return
    return queries;
  }

  /**
   * @brief ...
   */
  std::vector<std::string> PhysicalEntityManager::generateUpdateStateQueries(const std::string& inst_id, const json& dt_data)
  {
    // return value
    std::vector<std::string> queries;

    // - box_shape
    for (auto &m : mappings_box_shape_)
    {
      double val = extractValue(dt_data, m.jsonKey).get<double>();
      std::string q = "match $b isa " + getType() + ", has id \"" + inst_id + "\"; "
                            "(owner: $b, shape: $s) isa has_shape; "
                            "$s has " + m.attrName + " $s_a; "
                            "delete $s has $s_a; "
                      "insert $s has " + m.attrName + " " + std::to_string(val) + ";";
      queries.push_back(q);
    }

    // - position3d 
    for (auto &m : mappings_position3d_)
    {
      double val = extractValue(dt_data, m.jsonKey).get<double>();
      std::string q = "match $b isa " + getType() + ", has id \"" + inst_id + "\"; "
                            "(owner: $b, position: $p) isa has_position; "
                            "$p has " + m.attrName + " $p_a; "
                      "delete $p has $p_a; "
                      "insert $p has " + m.attrName + " " + std::to_string(val) + ";";
      queries.push_back(q);
    }

    // return
    return queries;
  }

}  // namespace anchoring_skrawl_plugins
