#include "ament_index_cpp/get_package_share_directory.hpp"

#include "anchoring_core/anchoring_manager.hpp"

#include <fstream>

namespace anchoring_skrawl_plugins
{

using json = nlohmann::json;

class PlaceSkillManager : public anchoring_core::AnchoringManager
{
public:
  using Ptr = std::shared_ptr<PlaceSkillManager>;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent) override
  {
      node_ = parent;
      auto node = parent.lock();
      logger_ = node->get_logger();
      RCLCPP_INFO(logger_, "Configuring plugin of type PlaceSkillManager");

      std::string pkg_share_dir =
        ament_index_cpp::get_package_share_directory("anchoring_skrawl_plugins");

      // Mapping rules
      std::ifstream fs(pkg_share_dir+"/skrawl/dt-mapping/rules_place_skill.json");
      json data = json::parse(fs);
      // - configure
      for (const auto &r : data)
      {
        mappings_place_def_.push_back({ r.value("json-path",""), r.value("target-attr","") });
      }

      RCLCPP_INFO(logger_, "Configured plugin of type PlaceSkillManager");
  }

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  void cleanup() override
  {
      RCLCPP_INFO(logger_, "Cleaning up plugin of type PlaceSkillManager");
  }

  /**
   * @brief Method to active the manager and any threads involved in execution.
   */
  void activate() override
  {
      RCLCPP_INFO(logger_, "Activating plugin of type PlaceSkillManager");
  }

  /**
   * @brief Method to deactivate the manager and any threads involved in execution.
   */
  void deactivate() override
  {
      RCLCPP_INFO(logger_, "Deactivating plugin of type PlaceSkillManager");
  }

  /**
   * @brief ...
   */
  std::vector<std::string> generatePopulateInstanceQueries(const json& elem) override
  {
    // return value
    std::vector<std::string> queries;

    // add object-specific processing of data
    auto id = elem.value("id", "");

    // affordances (conceptualize tasks afforded by objects - define tasks afforded by dispositions)
    std::string q = 
      "match $p  isa place, has id \""   + id     + "\"; "
            // concerned dispositions
            "$pd isa placeability; "
            "$sd isa support; "
            "$gd isa storage; "
            "$cp isa can_place; "
      "insert $pa1 (defines: $p, describes: $cp, describes: $pd, describes: $sd) isa place_affordance; "
             "$pa2 (defines: $p, describes: $cp, describes: $pd, describes: $gd) isa place_affordance; "
             "$s1 (is_context_for: $pa1) isa situation; "
             "$s2 (is_context_for: $pa2) isa situation; "
             // concerned roles
             "$pr isa placeable; "
             "$sr isa supporter; "
             "$cr isa container; "
             "$xr isa performer; "
             // role assignments for s1
             "(assigned_role: $pr, context: $s1) isa role_assignment;"  // placeable
             "(assigned_role: $sr, context: $s1) isa role_assignment;"  // supporter
             "(assigned_role: $xr, context: $s1) isa role_assignment;"  // performer
             // role assignments for s2
             "(assigned_role: $pr, context: $s2) isa role_assignment;"  // placeable
             "(assigned_role: $cr, context: $s2) isa role_assignment;"  // container
             "(assigned_role: $xr, context: $s2) isa role_assignment;"  // performer
             // preconditions
             "$absent isa required_is_in_front_of; "
             "(affordance: $pa1, required: $absent) isa affordance_precondition; "
             "(affordance: $pa2, required: $absent) isa affordance_precondition; ";
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

    // - place
    std::unordered_map<std::string, std::string> attr_values;
    for (auto &m : mappings_place_def_)
    {
      std::string val = extractValue(dt_data, m.jsonKey).get<std::string>();
      attr_values[m.attrName] = val;
    }
    if (attr_values["exec_result"].compare("success") != 0) {
      // Given wo (who) wt (what) and wr (where) as the participants in the place execution (1),
      // Find the role assignements for the situation with matching place affordances (2)
      // Update the player roles of the role assignments to point to wo, wh and wr (3)
      std::string q =
        "match $p  isa place, has id \"" + inst_id + "\"; "
              // get (i) entity id of who, (ii) the entity itself
              "$wo_dtc isa DigitalTwin_ID, has dt-id \"" + attr_values["who_id"] + "\"; "
              "$wo_hid (twin: $wo_dtc, asset: $wo) isa has_id; "
              "$wo isa physical_agent, has id $wo_id; "
              // get (i) entity id of what, (ii) the entity itself
              "$wt_dtc isa DigitalTwin_ID, has dt-id \"" + attr_values["what_id"] + "\"; "
              "$wt_hid (twin: $wt_dtc, asset: $wt) isa has_id; "
              "$wt isa physical_body, has id $wt_id; "
              // get (i) entity id of where, (ii) the entity itself
              "$wr_dtc isa DigitalTwin_ID, has dt-id \"" + attr_values["where_id"] + "\"; "
              "$wr_hid (twin: $wr_dtc, asset: $wr) isa has_id; "
              "$wr isa physical_entity, has id $wr_id; " // (1) done
              // get dispositions/capability of involved entities
              "(bearer: $wo, capability: $cwo)  isa has_capability;"
              "(bearer: $wt, disposition: $dwt) isa has_disposition;"
              "(bearer: $wr, disposition: $dwr) isa has_disposition;"
              // get the corresponding place affordance
              "$pa (defines: $p, describes: $cwo, describes: $dwt, describes: $dwr) isa place_affordance; "
              // get the situation that is context for the affordance
              "$s  (is_context_for: $pa) isa situation; "
              // and finally the role assignments
              "$rawo (assigned_role: $wor, context: $s) isa role_assignment;"
              "$wor isa performer; "
              "$rawt (assigned_role: $wtr, context: $s) isa role_assignment;"
              "$wtr isa placeable; "
              "$rawr (assigned_role: $wrr, context: $s) isa role_assignment; "
              "{ $wrr isa supporter; } or { $wrr isa container; };" // (2) done
        // update role assignments to refer the entities above who play the role of players
        "delete $rawo isa role_assignment; "
               "$rawt isa role_assignment; "
               "$rawr isa role_assignment; "
        "insert (assigned_role: $wor, player: $wo, context: $s) isa role_assignment; "
               "(assigned_role: $wtr, player: $wt, context: $s) isa role_assignment; "
               "(assigned_role: $wrr, player: $wr, context: $s) isa role_assignment;"; // (3) done
      queries.push_back(q);
    }
    // return
    return queries;
  }

protected:

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("PlaceSkillManager")};
  // parent node weak ptr
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // additional mappings for the entity
  std::vector<anchoring_core::AttributeMapping> mappings_place_def_;
};

}  // namespace anchoring_skrawl_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(anchoring_skrawl_plugins::PlaceSkillManager, anchoring_core::AnchoringManager)
