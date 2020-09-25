#include "ethercat_sdk_master/EthercatDevice.hpp"
#include "ethercat_sdk_master/EthercatMasterConfiguration.hpp"

#include <soem_interface/EthercatBusBase.hpp>

#include <memory>
#include <vector>

namespace ecat_master{

class EthercatMaster {
public:
  typedef std::shared_ptr<EthercatMaster> SharedPtr;
public:
  EthercatMaster() = default;
  void createEthercatBus();
  bool attachDevice(EthercatDevice::SharedPtr device);
  bool startup();
  bool startupStandalone();
  bool update();
  void shutdown();
  void preShutdown();

// Configuration
public:
  void loadEthercatMasterConfiguration(const EthercatMasterConfiguration& configuration);
  EthercatMasterConfiguration getConfiguration();

protected:
  std::unique_ptr<soem_interface::EthercatBusBase> bus_;
  std::vector<EthercatDevice::SharedPtr> devices_;
  EthercatMasterConfiguration configuration_;

protected:
  bool deviceExists(const std::string& name);
  void syncDistributedClock0(const std::vector<uint32_t>& addresses);


};
} // namespace ecat_master
