#ifndef ANCHORING_CORE__ANCHORING_MANAGER_HPP_
#define ANCHORING_CORE__ANCHORING_MANAGER_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <nlohmann/json.hpp>

namespace anchoring_core
{

using json = nlohmann::json;

/**
 * @brief Configuration for a DT mapping: maps a JSON key to a entity attribute
 */
struct AttributeMapping {
    /**
     * @brief Top-level key in DT JSON
     */
    std::string jsonKey;
    /**
     * @brief Attribute name in entity
     */
    std::string attrName;
};

class AnchoringManager
{
public:
  using Ptr = std::shared_ptr<AnchoringManager>;

  /**
   * @brief Virtual destructor
   */
  virtual ~AnchoringManager() {}

  /**
   * @param  parent pointer to user's node
   */
  virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active the manager and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactivate the manager and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief Generate domain-specific insert queries for populating the ontology with instances
   */
  virtual std::vector<std::string> generatePopulateInstanceQueries(const json& elem) = 0;

  /**
   * @brief Generate domain-specific insert queries for update instances' states in the ontology
   */
  virtual std::vector<std::string> generateUpdateStateQueries(const std::string& inst_id, const json& dt_data) = 0;

protected:

  /**
   * @brief Extract nested value from JSON using a dotted path with optional indices
   */
  double extractValue (const nlohmann::json &root, const std::string &path)
  {
    nlohmann::json cur = root;
    std::string token;
    std::istringstream ss(path);
    while (std::getline(ss, token, '.'))
    {
      auto idx0 = token.find('[');
      if (idx0 != std::string::npos)
      {
        auto key = token.substr(0, idx0);
        auto idx1 = token.find(']', idx0);
        int index = std::stoi(token.substr(idx0+1, idx1-idx0-1));
        cur = cur.at(key).at(index);
      }
      else
      {
        cur = cur.at(token);
      }
    }
    return cur.get<double>();
  }

  /**
   * @brief Container of entity attribute rules for mapping with DT info
   */
  std::vector<AttributeMapping> mappings_;

};

}  // namespace anchoring_core

#endif  // ANCHORING_CORE__ANCHORING_MANAGER_HPP_
