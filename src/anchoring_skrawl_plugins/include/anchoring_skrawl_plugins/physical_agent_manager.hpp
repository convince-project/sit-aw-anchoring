#ifndef ANCHORING_SKRAWL_PLUGINS__PHYSICAL_AGENT_MANAGER_HPP_
#define ANCHORING_SKRAWL_PLUGINS__PHYSICAL_AGENT_MANAGER_HPP_

#include "anchoring_skrawl_plugins/physical_entity_manager.hpp"

namespace anchoring_skrawl_plugins
{

class PhysicalAgentManager : public anchoring_skrawl_plugins::PhysicalEntityManager
{
public:
  using Ptr = std::shared_ptr<PhysicalAgentManager>;

  PhysicalAgentManager();

  std::vector<std::string> generatePopulateInstanceQueries(const anchoring_core::json& elem) override;

protected:

  virtual std::string getCapability() const = 0;

};

}  // namespace anchoring_skrawl_plugins

#endif  // ANCHORING_SKRAWL_PLUGINS__ANCHORING_SKRAWL_PHYSICAL_AGENT_MANAGER_HPP_
