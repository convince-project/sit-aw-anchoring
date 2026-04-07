#ifndef ANCHORING_SKRAWL_PLUGINS__ANCHORING_SKRAWL_PHYSICAL_OBJECT_MANAGER_HPP_
#define ANCHORING_SKRAWL_PLUGINS__ANCHORING_SKRAWL_PHYSICAL_OBJECT_MANAGER_HPP_

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "anchoring_core/anchoring_manager.hpp"

#include <fstream>

namespace anchoring_skrawl_plugins
{

using json = nlohmann::json;

class PhysicalObjectManager : public anchoring_core::AnchoringManager
{
public:
  using Ptr = std::shared_ptr<PhysicalObjectManager>;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent);
  void cleanup();
  void activate();
  void deactivate();
  virtual std::vector<std::string> generatePopulateInstanceQueries(const json& elem) = 0;
  virtual std::vector<std::string> generateUpdateStateQueries(const std::string& inst_id, const json& dt_data) = 0;

protected:

  virtual std::string getType() const = 0;
  virtual std::string getDisposition() const = 0;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("PhysicalObjectManager")};
  // parent node weak ptr
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // additional mappings for the entity
  std::vector<anchoring_core::AttributeMapping> mappings_box_shape_;
  std::vector<anchoring_core::AttributeMapping> mappings_position3d_;
};

}  // namespace anchoring_skrawl_plugins

#endif  // ANCHORING_SKRAWL_PLUGINS__ANCHORING_SKRAWL_PHYSICAL_OBJECT_MANAGER_HPP_
