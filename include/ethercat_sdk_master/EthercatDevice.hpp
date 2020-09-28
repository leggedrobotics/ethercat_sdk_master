#pragma once

#include <soem_interface/EthercatBusBase.hpp>
#include <soem_interface/EthercatSlaveBase.hpp>

#include <string>
#include <cstdint>
#include <memory>
namespace ecat_master {

/*!
 * Abstract class for EtherCAT devices compatible with the EthercatMaster.
 * EthercatDevice is derived from soem_interface.:EthercatSlaveBase.
 * The following functions must therefore also be implemented for every new device class:
 * - std::string getName() const;
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
   * Demand EtherCAT distributed clock synchronization for the device.
   * The distributed clock 0 is synchronized.
   * @return true if clock synchronization is required. (Default: false)
   */
  virtual bool clockSyncRequired(){return false;}

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



public:

  /*!
   * Send a write SDO of type Value to the device.
   * @note Using SDO and PDO communication simultaneously is usually a bad idea.
   * @param[in] index index of the SDO (16 bit).
   * @param[in] subindex sub index of the SDO (8 bit).
   * @param[in] completeAcces]ns true if all subindices are overwritten.
   * @param[in] value Value to be written.
   * @return true if successful.
   */
  template <typename Value>
  bool sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value value);

  /*!
   * Send a read SDO of type Value to the device.
   * @note Using SDO and PDO communication simultaneously is usually a bad idea.
   * @param[in] index index of the SDO (16 bit).
   * @param[in] subindex sub index of the SDO (8 bit).
   * @param[in] completeAcces]ns true if all subindices are read.
   * @param[out] value Value which is read from the device.
   * @return true if successful.
   */
  template <typename Value>
  bool sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value);

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
  bool sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value value, float delay = 0);

protected:
  std::string name_;
  double timeStep_{0.0};
};

// template specialization sdo writing
template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int8_t value);
template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int16_t value);
template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int32_t value);
template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int64_t value);
template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint8_t value);
template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint16_t value);
template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint32_t value);
template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint64_t value);
template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const float value);
template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const double value);

// template specialization sdo reading
template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t& value);
template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t& value);
template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t& value);
template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t& value);
template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t& value);
template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint16_t& value);
template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint32_t& value);
template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint64_t& value);
template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, float& value);
template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, double& value);
template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, std::string& value);

// template specialization sdo verivy write
template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t value, float delay);
template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t value, float delay);
template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t value, float delay);
template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t value, float delay);
template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t value, float delay);
template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint16_t value, float delay);
template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint32_t value, float delay);
template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint64_t value, float delay);
template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, float value, float delay);
template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, double value, float delay);

} // namespace ecat_master
