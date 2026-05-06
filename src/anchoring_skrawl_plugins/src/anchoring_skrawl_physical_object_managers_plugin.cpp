#include "anchoring_skrawl_plugins/physical_object_manager.hpp"

namespace anchoring_skrawl_plugins
{

class BlockManager : public PhysicalObjectManager
{

protected:

  /**
   * @brief ...
   */
  virtual std::string getType() const override
  {
    return("block");
  }

  /**
   * @brief ...
   */
  virtual std::vector<std::string> getDispositions() const override
  {
    return {
      "placeability",
      "support"
    };
  }
};

class SurfaceManager : public PhysicalObjectManager
{

protected:

  /**
   * @brief ...
   */
  virtual std::string getType() const override
  {
    return("surface");
  }

  /**
   * @brief ...
   */
  virtual std::vector<std::string> getDispositions() const override
  {
    return {
      "support"
    };
  }
};

class TableManager : public PhysicalObjectManager
{

protected:

  /**
   * @brief ...
   */
  virtual std::string getType() const override
  {
    return("table");
  }

  /**
   * @brief ...
   */
  virtual std::vector<std::string> getDispositions() const override
  {
    return{
      "support"
    };
  }
};

class StorageManager : public PhysicalObjectManager
{

protected:

  /**
   * @brief ...
   */
  virtual std::string getType() const override
  {
    return("storage_object");
  }

  /**
   * @brief ...
   */
  virtual std::vector<std::string> getDispositions() const override
  {
    return{
      "storage"
    };
  }
};

class SodaManager : public PhysicalObjectManager
{

protected:

  /**
   * @brief ...
   */
  virtual std::string getType() const override
  {
    return("soda");
  }

  /**
   * @brief ...
   */
  virtual std::vector<std::string> getDispositions() const override
  {
    return{
      "placeability"
    };
  }
};

class SnacksManager : public PhysicalObjectManager
{

protected:

  /**
   * @brief ...
   */
  virtual std::string getType() const override
  {
    return("snacks");
  }

  /**
   * @brief ...
   */
  virtual std::vector<std::string> getDispositions() const override
  {
    return{
      "placeability"
    };
  }
};

class ButterManager : public PhysicalObjectManager
{

protected:

  /**
   * @brief ...
   */
  virtual std::string getType() const override
  {
    return("butter");
  }

  /**
   * @brief ...
   */
  virtual std::vector<std::string> getDispositions() const override
  {
    return{
      "placeability"
    };
  }
};

}  // namespace anchoring_skrawl_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(anchoring_skrawl_plugins::BlockManager, anchoring_core::AnchoringManager)

PLUGINLIB_EXPORT_CLASS(anchoring_skrawl_plugins::SurfaceManager, anchoring_core::AnchoringManager)

PLUGINLIB_EXPORT_CLASS(anchoring_skrawl_plugins::TableManager, anchoring_core::AnchoringManager)

PLUGINLIB_EXPORT_CLASS(anchoring_skrawl_plugins::StorageManager, anchoring_core::AnchoringManager)

PLUGINLIB_EXPORT_CLASS(anchoring_skrawl_plugins::SodaManager, anchoring_core::AnchoringManager)

PLUGINLIB_EXPORT_CLASS(anchoring_skrawl_plugins::SnacksManager, anchoring_core::AnchoringManager)

PLUGINLIB_EXPORT_CLASS(anchoring_skrawl_plugins::ButterManager, anchoring_core::AnchoringManager)

