#include "ament_index_cpp/get_package_share_directory.hpp"

#include "anchoring_skrawl_plugins/physical_agent_manager.hpp"

#include <fstream>


namespace anchoring_skrawl_plugins
{

  using json = nlohmann::json;

  PhysicalAgentManager::PhysicalAgentManager()
    : PhysicalEntityManager("PhysicalAgentManager")
  {
  }

  /**
   * @brief ...
   */
  std::vector<std::string> PhysicalAgentManager::generatePopulateInstanceQueries(const json& elem)
  {
    // return value
    std::vector<std::string> queries;

    // Call base implementation
    queries = PhysicalEntityManager::generatePopulateInstanceQueries(elem);

    // - capabilities
    auto id = elem.value("id", "");
    std::string t = getType();
    for (auto c : getCapabilities())
    {
      // insert an agent capability only if it doesn't exist already
      std::string q;
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

}  // namespace anchoring_skrawl_plugins
