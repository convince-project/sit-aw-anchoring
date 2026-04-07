#include "anchoring_skrawl_plugins/physical_agent_manager.hpp"

namespace anchoring_skrawl_plugins
{

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
  virtual std::string getCapability() const override
  {
    return("can_place");
  }
};

}  // namespace anchoring_skrawl_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(anchoring_skrawl_plugins::RobotManager, anchoring_core::AnchoringManager)
