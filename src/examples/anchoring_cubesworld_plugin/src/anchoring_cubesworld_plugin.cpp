#include "ament_index_cpp/get_package_share_directory.hpp"

#include "anchoring_core/anchoring_manager.hpp"

#include <fstream>

namespace anchoring_cubesworld_plugin
{

using json = nlohmann::json;

class CubeManager : public anchoring_core::AnchoringManager
{
public:
  using Ptr = std::shared_ptr<CubeManager>;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent) override
  {
      node_ = parent;
      auto node = parent.lock();
      logger_ = node->get_logger();
      RCLCPP_INFO(logger_, "Configuring plugin of type CubeManager");

      std::string pkg_share_dir =
        ament_index_cpp::get_package_share_directory("anchoring_cubesworld_plugin");

      // Mapping rules
      std::ifstream fc(pkg_share_dir+"/rules/mapping/rules_cube.json");
      json data = json::parse(fc);
      // - configure
      for (const auto &r : data)
      {
        mappings_.push_back({ r.value("json-path",""), r.value("target-attr","") });
      }

      std::ifstream fp(pkg_share_dir+"/rules/mapping/rules_pose.json");
      data = json::parse(fp);
      // - configure
      for (const auto &r : data)
      {
        mappings_pose_.push_back({ r.value("json-path",""), r.value("target-attr","") });
      }
      RCLCPP_INFO(logger_, "Configured plugin of type CubeManager");
  }

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  void cleanup() override
  {
      RCLCPP_INFO(logger_, "Cleaning up plugin of type CubeManager");
  }

  /**
   * @brief Method to active the manager and any threads involved in execution.
   */
  void activate() override
  {
      RCLCPP_INFO(logger_, "Activating plugin of type CubeManager");
  }

  /**
   * @brief Method to deactivate the manager and any threads involved in execution.
   */
  void deactivate() override
  {
      RCLCPP_INFO(logger_, "Deactivating plugin of type CubeManager");
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

    // - default values of cube position
    std::string q = "match $a isa Cube, has id \""   + id     + "\"; "
                    "insert $a has pos_x 0.0, has pos_y 0.0, has pos_z 0.0, "
                              "has box_height 0.0, has box_width 0.0, has box_length 0.0; ";
    queries.push_back(q);

    // - grasp pose (with default values)
    q = "match $a isa Cube, has id \""   + id     + "\"; "
        "insert $p isa Pose, has pos_x 0.0, has pos_y 0.0, has pos_z 0.0; "
        "(cube: $a, grasp_pose: $p) isa has_grasp_pose; ";
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
      std::string q = "match $c isa Cube, has id \"" + inst_id + "\", has " + m.attrName + " $c_a; "
                      "delete $c has $c_a; "
                      "insert $c has " + m.attrName + " " + std::to_string(val) + ";";
      queries.push_back(q);
    }

    for (auto &m : mappings_pose_)
    {
      double val = extractValue(dt_data, m.jsonKey);
      std::string q = "match $c isa Cube, has id \"" + inst_id + "\"; "
                      "(cube: $c, grasp_pose: $p) isa has_grasp_pose; "
                      "$p has " + m.attrName + " $p_a; "
                      "delete $p has $p_a; "
                      "insert $p has " + m.attrName + " " + std::to_string(val) + ";";
      queries.push_back(q);
    }

    // return
    return queries;
  }

protected:

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("CubeManager")};
  // parent node weak ptr
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // FIXME remove to use specific entity manager
  std::vector<anchoring_core::AttributeMapping> mappings_pose_;
};

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
        ament_index_cpp::get_package_share_directory("anchoring_cubesworld_plugin");

      // Mapping rules
      std::ifstream f(pkg_share_dir+"/rules/mapping/rules_gripper.json");
      json data = json::parse(f);
      // - configure
      for (const auto &r : data)
      {
        mappings_.push_back({ r.value("json-path",""), r.value("target-attr","") });
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
    std::string q = "match $a isa Agent, has id \""   + id     + "\"; "
                    "insert $a has pos_x 0.0, has pos_y 0.0, has pos_z 0.0, "
                              "has remaining_reach 0.0; ";
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
      std::string q = "match $a isa Agent, has id \"" + inst_id + "\", has " + m.attrName + " $a_a; "
                      "delete $a has $a_a; "
                      "insert $a has " + m.attrName + " " + std::to_string(val) + ";";
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
};

}  // namespace anchoring_cubesworld_plugin

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(anchoring_cubesworld_plugin::CubeManager, anchoring_core::AnchoringManager)
PLUGINLIB_EXPORT_CLASS(anchoring_cubesworld_plugin::GripperManager, anchoring_core::AnchoringManager)
