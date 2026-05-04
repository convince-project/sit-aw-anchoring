#include "ament_index_cpp/get_package_share_directory.hpp"

#include "anchoring_skrawl_plugins/physical_agent_manager.hpp"

#include <fstream>

namespace anchoring_skrawl_plugins
{

using json = nlohmann::json;

class RobotManager : public PhysicalAgentManager
{

protected:

  /**
   * @brief ...
   */
  virtual std::string getType() const override
  {
    return("robot");
  }

  /**
   * @brief ...
   */
  virtual std::vector<std::string> getCapabilities() const override
  {
    return{
      "can_place"
    };
  }
};

class GripperManager : public PhysicalAgentManager
{

public:

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent) override
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

//      std::ifstream fg(pkg_share_dir+"/skrawl/dt-mapping/rules_gripper.json");
//      data = json::parse(fg);
//      // - configure
//      for (const auto &r : data)
//      {
//        mappings_.push_back({ r.value("json-path",""), r.value("target-attr","") });
//      }

      std::ifstream fp(pkg_share_dir+"/skrawl/dt-mapping/rules_gripper_tcp.json");
      data = json::parse(fp);
      // - configure
      for (const auto &r : data)
      {
        mappings_tcp_.push_back({ r.value("json-path",""), r.value("target-attr","") });
      }

      RCLCPP_INFO(logger_, "Configured plugin of type %s", getType().c_str());
  }

  std::vector<std::string> generatePopulateInstanceQueries(const json& elem) override
  {
    // return value
    std::vector<std::string> queries;

    // add physical object-specific processing of data
    auto id = elem.value("id", "");

    // From base implementation
    // - box shape (with default values)
    std::string q = "match $a isa " + getType() + ", has id \""   + id     + "\"; "
                    "insert $bs isa box_shape, has height 0.0, has width 0.0, has length 0.0; "
                           "(owner: $a, shape: $bs) isa has_shape; ";
    queries.push_back(q);

    // - tcp pose (with default values)
    q = "match $a isa gripper, has id \""   + id     + "\"; "
        "insert $p isa pose3d; "
               "$t isa position3d, has lin_x 0.0, has lin_y 0.0, has lin_z 0.0; "
               "$r isa quaternion, has rot_x 0.0, has rot_y 0.0, has rot_z 0.0, has rot_w 1.0; "
               "(pose: $p, translation: $t, rotation: $r) isa pose_description; "
               "(owner: $a, pose: $p) isa has_tool_center_pose; ";
    queries.push_back(q);

    // - capabilities
    std::string t = getType();
    for (auto c : getCapabilities())
    {
      // insert an agent capability only if it doesn't exist already
      q = "match $a isa " + t + ", has id \""   + id     + "\"; "
                "$t type " + c + "; "
                "not { $x isa $t; }; "
          "insert $c isa $t; ";
      queries.push_back(q);
      q = "match $a isa " + t + ", has id \""   + id     + "\"; "
                "$c isa " + c + "; "
          "insert (bearer: $a, capability: $c) isa has_capability; ";
      queries.push_back(q);
    }

    // return
    return queries;
  }

  std::vector<std::string> generateUpdateStateQueries(const std::string& inst_id, const json& dt_data) override
  {
    // return value
    std::vector<std::string> queries;

//    for (auto &m : mappings_)
//    {
//      double val = extractValue(dt_data, m.jsonKey);
//      std::string q = "match $a isa gripper, has id \"" + inst_id + "\", has " + m.attrName + " $a_a; "
//                      "delete $a has $a_a; "
//                      "insert $a has " + m.attrName + " " + std::to_string(val) + ";";
//      queries.push_back(q);
//    }

    // From base implementation
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

  /**
   * @brief ...
   */
  virtual std::string getType() const override
  {
    return("gripper");
  }

  /**
   * @brief ...
   */
  virtual std::vector<std::string> getCapabilities() const override
  {
    return{
      "can_place"
    };
  }

  std::vector<anchoring_core::AttributeMapping> mappings_tcp_;
};

}  // namespace anchoring_skrawl_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(anchoring_skrawl_plugins::RobotManager,   anchoring_core::AnchoringManager)
PLUGINLIB_EXPORT_CLASS(anchoring_skrawl_plugins::GripperManager, anchoring_core::AnchoringManager)
