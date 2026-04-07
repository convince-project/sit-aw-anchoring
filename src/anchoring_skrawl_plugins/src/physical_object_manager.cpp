#include "ament_index_cpp/get_package_share_directory.hpp"

#include "anchoring_skrawl_plugins/physical_object_manager.hpp"

#include <fstream>


namespace anchoring_skrawl_plugins
{

  using json = nlohmann::json;

  PhysicalObjectManager::PhysicalObjectManager()
    : PhysicalEntityManager("PhysicalObjectManager")
  {
  }

  /**
   * @brief ...
   */
  std::vector<std::string> PhysicalObjectManager::generatePopulateInstanceQueries(const json& elem)
  {
    // return value
    std::vector<std::string> queries;

    // Call base implementation
    queries = PhysicalEntityManager::generatePopulateInstanceQueries(elem);

    // - disposition (insert an object disposition only if it doesn't exist already)
    auto id = elem.value("id", "");
    std::string q;
    q = "match $a isa " + getType() + ", has id \""   + id     + "\"; "
              "$t type " + getDisposition() + "; "
              "not { $x isa $t; }; "
        "insert $d isa $t; ";
    queries.push_back(q);
    q = "match $a isa " + getType() + ", has id \""   + id     + "\"; "
              "$d isa " + getDisposition() + "; "
        "insert (bearer: $a, disposition: $d) isa has_disposition; ";
    queries.push_back(q);

    // return
    return queries;
  }

}  // namespace anchoring_skrawl_plugins
