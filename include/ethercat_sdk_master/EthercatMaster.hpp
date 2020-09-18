#include "ethercat_sdk_master/EthercatDrive.hpp"
#include "ethercat_sdk_master/EthercatMasterConfiguration.hpp"

#include <soem_interface/EthercatBusBase.hpp>

#include <memory>
#include <vector>

namespace ecat_master{

class EthercatMaster {
public:
  EthercatMaster() = default;
  void createEthercatBus();
  bool attachDrive(std::shared_ptr<EthercatDrive> drive);
  bool startup();
  bool update();
  void shutdown();

// Configuration
public:
  void loadEthercatMasterConfiguration(const EthercatMasterConfiguration& configuration);

protected:
  std::unique_ptr<soem_interface::EthercatBusBase> bus_;
  std::vector<std::shared_ptr<EthercatDrive>> drives_;
  EthercatMasterConfiguration configuration_;

protected:
  bool driveExists(const std::string& name);
  void setEthercatState(ec_state state, const std::vector<uint32_t>& addresses);
  void syncDistributedClock0(const std::vector<uint32_t>& addresses);


};
} // namespace ecat_master
