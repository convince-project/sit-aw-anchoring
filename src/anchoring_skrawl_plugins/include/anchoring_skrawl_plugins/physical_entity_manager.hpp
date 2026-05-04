#ifndef ANCHORING_SKRAWL_PLUGINS__PHYSICAL_ENTITY_MANAGER_HPP_
#define ANCHORING_SKRAWL_PLUGINS__PHYSICAL_ENTITY_MANAGER_HPP_

#include "anchoring_core/anchoring_manager.hpp"

namespace anchoring_skrawl_plugins
{

class PhysicalEntityManager : public anchoring_core::AnchoringManager
{
public:
  using Ptr = std::shared_ptr<PhysicalEntityManager>;

  explicit PhysicalEntityManager(const std::string& logger_name)
    : logger_{rclcpp::get_logger(logger_name)}{}

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  std::vector<std::string> generatePopulateInstanceQueries(const anchoring_core::json& elem) override;
  std::vector<std::string> generateUpdateStateQueries(const std::string& inst_id,
                                                      const anchoring_core::json& dt_data) override;

protected:

  virtual std::string getType() const = 0;

  // Logger
  rclcpp::Logger logger_;
  // parent node weak ptr
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // additional mappings for the entity
  std::vector<anchoring_core::AttributeMapping> mappings_box_shape_;
  std::vector<anchoring_core::AttributeMapping> mappings_position3d_;
};

}  // namespace anchoring_skrawl_plugins

#endif  // ANCHORING_SKRAWL_PLUGINS__ANCHORING_SKRAWL_PHYSICAL_ENTITY_MANAGER_HPP_
