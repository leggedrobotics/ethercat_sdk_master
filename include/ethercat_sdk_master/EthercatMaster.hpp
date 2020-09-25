#include "ethercat_sdk_master/EthercatDevice.hpp"
#include "ethercat_sdk_master/EthercatMasterConfiguration.hpp"
#include "ethercat_sdk_master/UpdateMode.hpp"

#include <soem_interface/EthercatBusBase.hpp>

#include <memory>
#include <vector>
#include <chrono>

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
  bool update(UpdateMode updateMode);
  void shutdown();
  void preShutdown();
  soem_interface::EthercatBusBase* getBusPtr() { return bus_.get(); }

// Configuration
public:
  void loadEthercatMasterConfiguration(const EthercatMasterConfiguration& configuration);
  EthercatMasterConfiguration getConfiguration();

protected:
  std::unique_ptr<soem_interface::EthercatBusBase> bus_{nullptr};
  std::vector<EthercatDevice::SharedPtr> devices_;
  EthercatMasterConfiguration configuration_;
  std::chrono::time_point<std::chrono::high_resolution_clock> lastWakeupTime_;
  std::chrono::high_resolution_clock::duration updateDuration_;
  std::chrono::high_resolution_clock::duration deltaT_;
  std::chrono::high_resolution_clock::duration targetUpdateDuration_;
  unsigned int rateTooLowCounter_{0};

protected:
  bool deviceExists(const std::string& name);
  void syncDistributedClock0(const std::vector<uint32_t>& addresses);
  void createUpdateHeartbeat(bool enforceRate);


};
} // namespace ecat_master
