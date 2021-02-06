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

#pragma once

#include <soem_interface/EthercatBusBase.hpp>
#include <soem_interface/EthercatSlaveBase.hpp>

#include <string>
#include <cstdint>
#include <memory>
#include <thread>
#include <chrono>
namespace ecat_master {

/*!
 * Abstract class for EtherCAT devices compatible with the EthercatMaster.
 * EthercatDevice is derived from soem_interface.:EthercatSlaveBase.
 * The following functions must therefore also be implemented for every new device class:
 * - bool startup();
 * - void updateRead();
 * - void updateWrite();
 * - void shutdown();
 * - PdoInfo getCurrentPdoInfo() const;
 */
class EthercatDevice : public soem_interface::EthercatSlaveBase{
public:
  typedef std::shared_ptr<EthercatDevice> SharedPtr;
public:

public:

  /*!
   * Set the update time step.
   * This value needs to correspond to the target time step used in the update
   * of the EtherCAT communication.
   * You don't have to update this value in the communication update loop.
   * This function does not have to be called manually when using the EthercatMaster.
   * @param[in] timeStep communication update time step.
   */
  virtual void setTimeStep(double timeStep);

  /*!
   * PDO preparation for communication shutdown.
   * Things that need to be done before the PDO communication stops.
   * This function must be called in a thread separated from the communication
   * update thread!
   * The PDO communication will continue during the execution of this function.
   * Pay attention to not block updateRead or updateWrite with mutexes and don't
   * create data races.
   * This function should only return once the desired state of the drive has been reached.
   * Look at the AnydriveEthercatSlave class in the anydrive_sdk for an example.
   */
  virtual void preShutdown() {};

  /*!
   * Return the name of this device.
   */
  virtual std::string getName() const override {return name_;}

  /*!
   * Set the name of the device.
   * @param[in] name Name of device.
   */
  virtual void setName(const std::string& name) {name_ = name;}


public:

  /*!
   * Send a write SDO of type Value to the device and confirm by reading
   * the same value afterwards.
   * @note Using SDO and PDO communication simultaneously is usually a bad idea.
   * @param[in] index index of the SDO (16 bit).
   * @param[in] subindex sub index of the SDO (8 bit).
   * @param[in] completeAcces]ns true if all subindices are read.
   * @param[in] value Value which is read from the device.
   * @param[in] delay Delay between the writing and reading SDO calls (in microseconds).
   * @return true if the read value corresponds to the written value.
   */
  template <typename Value>
  bool sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value value, float delay = 0) {
    Value testVal;
    bool success = true;
    success &= sendSdoWrite(index, subindex, completeAccess, value);
    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(delay)));
    success &= sendSdoRead(index, subindex, completeAccess, testVal);
    return (success & (value == testVal));
  }

protected:
  std::string name_;
  double timeStep_{0.0};
};

} // namespace ecat_master
