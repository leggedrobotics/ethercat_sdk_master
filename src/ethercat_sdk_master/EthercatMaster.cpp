#include "ethercat_sdk_master/EthercatMaster.hpp"

#include <thread>

namespace ecat_master{

void EthercatMaster::loadEthercatMasterConfiguration(const EthercatMasterConfiguration &configuration){
  using namespace std::chrono_literals;
  devices_.clear();
  configuration_ = configuration;

  // set durations and timepoints
  targetUpdateDuration_ = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
    std::chrono::duration<double>(configuration.timeStep));
  updateDuration_ = 0s;
  deltaT_ = 0s;
  lastWakeupTime_ = std::chrono::high_resolution_clock::now() +
    std::chrono::duration<int>(std::numeric_limits<int>::max());
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

bool EthercatMaster::update(UpdateMode updateMode){
  bus_->updateWrite();
  bus_->updateRead();

  // create update heartbeat if in standalone mode
  switch(updateMode){
    case UpdateMode::StandaloneEnforceRate:
      createUpdateHeartbeat(true);
      break;
    case UpdateMode::StandaloneEnforceStep:
      createUpdateHeartbeat(false);
      break;
    case UpdateMode::NonStandalone:
      break;
  }
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

void EthercatMaster::createUpdateHeartbeat(bool enforceRate){
  using namespace std::chrono;
  using namespace std::chrono_literals;
  // trying to keep the rate constant but not necessarily the update time step
  updateDuration_ = high_resolution_clock::now() - lastWakeupTime_;
  if(enforceRate){
    // Rate is enforced
    deltaT_ = targetUpdateDuration_ - updateDuration_ + deltaT_;
    if(deltaT_.count() > 0){ // we are early
      if(deltaT_ > targetUpdateDuration_)
        std::this_thread::sleep_for(targetUpdateDuration_); // happens on first startup
      else
        std::this_thread::sleep_for(deltaT_);
      deltaT_ = 0s;
      rateTooLowCounter_ = 0;
    } else { // "We are late"
      rateTooLowCounter_++;
    }
  } else {
    // Rate is not enforced
    deltaT_ = targetUpdateDuration_ - updateDuration_;
    if(deltaT_.count() > 0){
      if(deltaT_ > targetUpdateDuration_)
        std::this_thread::sleep_for(targetUpdateDuration_); // happens on first startup
      else
        std::this_thread::sleep_for(deltaT_);
      rateTooLowCounter_ = 0;
    } else {
      rateTooLowCounter_++;
    }
  }
  if(rateTooLowCounter_ >= configuration_.updateRateTooLowWarnThreshold){
    rateTooLowCounter_ = 0;
    MELO_WARN("[ethercat_sdk_master:EthercatMaster::update] processing took too long.");
  }
  lastWakeupTime_ = high_resolution_clock::now();
}

} // namespace ecat_master
