#include "ethercat_sdk_master/EthercatMaster.hpp"

#include <unistd.h>

namespace ecat_master{

void EthercatMaster::loadEthercatMasterConfiguration(const EthercatMasterConfiguration &configuration){
  drives_.clear();
  configuration_ = configuration;
  createEthercatBus();
}

void EthercatMaster::createEthercatBus(){
  bus_.reset(new soem_interface::EthercatBusBase(configuration_.networkInterface));
}

// According to <https://infosys.beckhoff.com/index.php?content=../content/1031/ax2000-b110/html/bt_ecbasics_ecstatemachine.htm>
void EthercatMaster::setEthercatState(ec_state state, const std::vector<uint32_t>& addresses){
  switch(state){
    case EC_STATE_INIT:
      for(const auto& address : addresses){
        bus_->setState(EC_STATE_INIT, address);
      }
      for(const auto& address : addresses){
        bus_->waitForState(EC_STATE_INIT, address, 50, 0.05);
      }
      break;
    case EC_STATE_PRE_OP:
      for(const auto& address : addresses){
        bus_->setState(EC_STATE_INIT, address);
      }
      for(const auto& address : addresses){
        bus_->waitForState(EC_STATE_INIT, address, 50, 0.05);
      }
      usleep(configuration_.ethercatStateChangeTimeout);
      for(const auto& address : addresses){
        bus_->setState(EC_STATE_PRE_OP, address);
      }
      for(const auto& address : addresses){
        bus_->waitForState(EC_STATE_PRE_OP, address, 50, 0.05);
      }
      break;
    case EC_STATE_OPERATIONAL:
      for(const auto& address : addresses){
        bus_->setState(EC_STATE_INIT, address);
      }
      for(const auto& address : addresses){
        bus_->waitForState(EC_STATE_INIT, address, 50, 0.05);
      }
      usleep(configuration_.ethercatStateChangeTimeout);
      for(const auto& address : addresses){
        bus_->setState(EC_STATE_PRE_OP, address);
      }
      for(const auto& address : addresses){
        bus_->waitForState(EC_STATE_PRE_OP, address, 50, 0.05);
      }
      usleep(configuration_.ethercatStateChangeTimeout);
      for(const auto& address : addresses){
        bus_->setState(EC_STATE_SAFE_OP, address);
      }
      for(const auto& address : addresses){
        bus_->waitForState(EC_STATE_SAFE_OP, address, 50, 0.05);
      }
      usleep(configuration_.ethercatStateChangeTimeout);
      for(const auto& address : addresses){
        bus_->setState(EC_STATE_OPERATIONAL, address);
      }
      for(const auto& address : addresses){
        bus_->waitForState(EC_STATE_OPERATIONAL, address, 50, 0.05);
      }
      break;
    default:
      std::cout << "Requested EtherCAT state is not implemented" << std::endl;
  }
}
void EthercatMaster::syncDistributedClock0(const std::vector<uint32_t>& addresses){
  for(const auto& address: addresses){
    bus_->syncDistributedClock0(address, true, configuration_.timeStep, configuration_.timeStep / 2.f);
  }
}

bool EthercatMaster::attachDrive(std::shared_ptr<EthercatDrive> drive){
  if (driveExists(drive->getName())){
    std::cout << "Cannot attach drive with name '" << drive->getName()
              << "' because it already exists." << std::endl;
    return false;
  }
  bus_->addSlave(drive);
  drive->setEthercatBusBasePointer(bus_.get());
  drives_.push_back(drive);
  std::cout << "Attached drive '"
            << drive->getName()
            << "' to address "
            << drive->getAddress()
            << std::endl;
  return true;
}

bool EthercatMaster::startup(){
  bool success = true;

  // Drives requiring clock synchronization and/or online preop config
  std::vector<uint32_t> addressesWithClockSync;
  std::vector<uint32_t> addressesWithPreopConfig;
  std::vector<std::shared_ptr<EthercatDrive>> drivesWithPreopConfig;
  for(const auto& drive : drives_){
    if(drive->clockSyncRequired())
      addressesWithClockSync.push_back(drive->getAddress());
    if(drive->preopConfigurationRequired()){
      addressesWithPreopConfig.push_back(drive->getAddress());
      drivesWithPreopConfig.push_back(drive);
    }
  }
  success &= bus_->startup(false); // TODO solve this size_check issue
  if(!success)
    return false;
  syncDistributedClock0(addressesWithClockSync);
  setEthercatState(EC_STATE_PRE_OP, addressesWithPreopConfig);
  for(const auto& drive: drivesWithPreopConfig){
    drive->runPreopConfiguration();
  }
  setEthercatState(EC_STATE_INIT, std::vector<uint32_t>{0});
  bus_->shutdown(); // TODO: Check if required
  usleep(1000000);
  bus_->startup(false); // TODO: solve size_check issue
  syncDistributedClock0(addressesWithClockSync);
  setEthercatState(EC_STATE_OPERATIONAL, std::vector<uint32_t>{0});

  for(const auto& drive : drives_){
    success &= drive->startup();
  }
  return success;
}

bool EthercatMaster::update(){
  bus_->updateWrite();
  bus_->updateRead();
}

void EthercatMaster::shutdown(){
  bus_->setState(EC_STATE_INIT);
  bus_->shutdown();
}

bool EthercatMaster::driveExists(const std::string& name){
  for (const auto& drive : drives_) {
    if (drive->getName() == name) {
      return true;
    }
  }
  return false;
}
} // namespace ecat_master
