#include "ament_index_cpp/get_package_share_directory.hpp"

#include "anchoring_core/anchoring_manager.hpp"

#include <fstream>

namespace anchoring_skrawl_plugins
{

using json = nlohmann::json;

class GripperManager : public anchoring_core::AnchoringManager
{
public:
  using Ptr = std::shared_ptr<GripperManager>;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent) override
  {
      node_ = parent;
      auto node = parent.lock();
      logger_ = node->get_logger();
      RCLCPP_INFO(logger_, "Configuring plugin of type GripperManager");

      std::string pkg_share_dir =
        ament_index_cpp::get_package_share_directory("anchoring_skrawl_plugins");

      // Mapping rules
      std::ifstream f(pkg_share_dir+"/skrawl/dt-mapping/rules_gripper.json");
      json data = json::parse(f);
      // - configure
      for (const auto &r : data)
      {
        mappings_.push_back({ r.value("json-path",""), r.value("target-attr","") });
      }

      std::ifstream fp(pkg_share_dir+"/skrawl/dt-mapping/rules_gripper_tcp.json");
      data = json::parse(fp);
      // - configure
      for (const auto &r : data)
      {
        mappings_tcp_.push_back({ r.value("json-path",""), r.value("target-attr","") });
      }
      RCLCPP_INFO(logger_, "Configured plugin of type GripperManager");
  }

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  void cleanup() override
  {
      RCLCPP_INFO(logger_, "Cleaning up plugin of type GripperManager");
  }

  /**
   * @brief Method to active the manager and any threads involved in execution.
   */
  void activate() override
  {
      RCLCPP_INFO(logger_, "Activating plugin of type GripperManager");
  }

  /**
   * @brief Method to deactivate the manager and any threads involved in execution.
   */
  void deactivate() override
  {
      RCLCPP_INFO(logger_, "Deactivating plugin of type GripperManager");
  }

  /**
   * @brief ...
   */
  std::vector<std::string> generatePopulateInstanceQueries(const json& elem) override
  {
    // return value
    std::vector<std::string> queries;

    // add cube-specific processing of data
    auto id = elem.value("id", "");

    // - default values of gripper center
    std::string q = "match $a isa gripper, has id \""   + id     + "\"; "
                    "insert $a has remaining_reach 0.0; ";
    queries.push_back(q);

    // - tcp pose (with default values)
    q = "match $a isa gripper, has id \""   + id     + "\"; "
        "insert $p isa pose3d; "
               "$t isa position3d, has lin_x 0.0, has lin_y 0.0, has lin_z 0.0; "
               "$r isa quaternion, has rot_x 0.0, has rot_y 0.0, has rot_z 0.0, has rot_w 1.0; "
               "(pose: $p, translation: $t, rotation: $r) isa pose_description; "
               "(owner: $a, pose: $p) isa has_tool_center_pose; ";
    queries.push_back(q);

    // return
    return queries;
  }

  /**
   * @brief ...
   */
  virtual std::vector<std::string> generateUpdateStateQueries(const std::string& inst_id, const json& dt_data) override
  {
    // return value
    std::vector<std::string> queries;

    for (auto &m : mappings_)
    {
      double val = extractValue(dt_data, m.jsonKey);
      std::string q = "match $a isa gripper, has id \"" + inst_id + "\", has " + m.attrName + " $a_a; "
                      "delete $a has $a_a; "
                      "insert $a has " + m.attrName + " " + std::to_string(val) + ";";
      queries.push_back(q);
    }

    for (auto &m : mappings_tcp_)
    {
      double val = extractValue(dt_data, m.jsonKey);
      std::string q = "match $a isa gripper, has id \"" + inst_id + "\"; "
                            "(owner: $a, pose: $p) isa has_tool_center_pose; "
                            "(pose: $p, translation: $t, rotation: $r) isa pose_description; "
                            "$t has " + m.attrName + " $t_a; "
                      "delete $t has $t_a; "
                      "insert $t has " + m.attrName + " " + std::to_string(val) + ";";
      queries.push_back(q);
    }

    // return
    return queries;
  }

protected:

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("GripperManager")};
  // parent node weak ptr
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // FIXME remove to use specific entity manager
  std::vector<anchoring_core::AttributeMapping> mappings_tcp_;
};

}  // namespace anchoring_skrawl_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(anchoring_skrawl_plugins::GripperManager, anchoring_core::AnchoringManager)
