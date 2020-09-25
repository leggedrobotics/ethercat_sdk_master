#include "ethercat_sdk_master/EthercatMaster.hpp"

#include <unistd.h>

namespace ecat_master{

void EthercatMaster::loadEthercatMasterConfiguration(const EthercatMasterConfiguration &configuration){
  devices_.clear();
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
    bus_->syncDistributedClock0(address, true, configuration_.timeStep, configuration_.timeStep / 2.f);
  }
}

bool EthercatMaster::attachDevice(EthercatDevice::SharedPtr device){
  if (deviceExists(device->getName())){
    std::cout << "Cannot attach device with name '" << device->getName()
              << "' because it already exists." << std::endl;
    return false;
  }
  bus_->addSlave(device);
  device->setEthercatBusBasePointer(bus_.get());
  device->setTimeStep(configuration_.timeStep);
  devices_.push_back(device);
  std::cout << "Attached device '"
            << device->getName()
            << "' to address "
            << device->getAddress()
            << std::endl;
  return true;
}

bool EthercatMaster::startup(){
  bool success = true;
  success &= bus_->startup(false);

  for(const auto & device : devices_)
  {
    if(!bus_->waitForState(EC_STATE_SAFE_OP, device->getAddress(), 50, 0.05))
      std::cout << "not in safe op after satrtup!" << std::endl;
    bus_->setState(EC_STATE_OPERATIONAL, device->getAddress());
    success &= bus_->waitForState(EC_STATE_OPERATIONAL, device->getAddress(), 50, 0.05);
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
   for(auto & device : devices_)
      device->preShutdown();
}

bool EthercatMaster::deviceExists(const std::string& name){
  for (const auto& device : devices_) {
    if (device->getName() == name) {
      return true;
    }
  }
  return false;
}
} // namespace ecat_master
