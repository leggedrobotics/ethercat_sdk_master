#include "ethercat_sdk_master/EthercatMaster.hpp"

#include <unistd.h>

namespace ecat_master{

void EthercatMaster::loadEthercatMasterConfiguration(const EthercatMasterConfiguration &configuration){
  drives_.clear();
  configuration_ = configuration;
  createEthercatBus();
}

EthercatMasterConfiguration EthercatMaster::getConfiguration()
{
    return configuration_;
}

void EthercatMaster::createEthercatBus(){
    bus_.reset(new soem_interface::EthercatBusBase(configuration_.networkInterface));
}

void EthercatMaster::syncDistributedClock0(const std::vector<uint32_t>& addresses){
  for(const auto& address: addresses){
    //bus_->syncDistributedClock0(address, true, configuration_.timeStep, configuration_.timeStep / 2.f);
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
  drive->setTimeStep(configuration_.timeStep);
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
  success &= bus_->startup(false);

  for(const auto & slave: drives_)
  {
      bus_->waitForState(EC_STATE_SAFE_OP, slave->getAddress(), 50, 0.05);
      bus_->setState(EC_STATE_OPERATIONAL, slave->getAddress());
      success &= bus_->waitForState(EC_STATE_OPERATIONAL, slave->getAddress(), 50, 0.05);
  }


  MELO_DEBUG_STREAM( "EthercatMaster::startup() success: " << success);
  return success;

}

// TODO: implement
bool EthercatMaster::startupStandalone(){
  return true;
}

bool EthercatMaster::update(){
  bus_->updateWrite();
  bus_->updateRead();
  return true;
}

void EthercatMaster::shutdown(){
  bus_->setState(EC_STATE_INIT);
  bus_->shutdown();
}

void EthercatMaster::preShutdown()
{
   for(auto & slave: drives_)
      slave->preShutdown();
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
