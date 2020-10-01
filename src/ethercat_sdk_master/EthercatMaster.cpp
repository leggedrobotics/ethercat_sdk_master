
/*
 ** Copyright 2020 Robotic Systems Lab - ETH Zurich:
 ** Lennart Nachtigall, Jonas Junger
 ** Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 **
 ** 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 **
 ** 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 **
 ** 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 **
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


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

void EthercatMaster::update(UpdateMode updateMode){
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
