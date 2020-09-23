#pragma once

#include <soem_interface/EthercatBusBase.hpp>
#include <soem_interface/EthercatSlaveBase.hpp>

#include <string>
#include <cstdint>
#include <memory>
namespace ecat_master {
class EthercatDrive : public soem_interface::EthercatSlaveBase{
public:
  virtual void shutdown() = 0;

public:
  virtual bool clockSyncRequired(){return false;}
  virtual void setTimeStep(double timeStep);
  virtual void preShutdown() {};

public:
  template <typename Value>
  bool sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value value);

  template <typename Value>
  bool sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value);

  template <typename Value>
  bool sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value value, float delay = 0);

protected:
  std::string name_;
  double timeStep_{0.0};
};

// template specialization sdo writing
template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int8_t value);
template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int16_t value);
template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int32_t value);
template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int64_t value);
template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint8_t value);
template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint16_t value);
template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint32_t value);
template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint64_t value);
template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const float value);
template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const double value);

// template specialization sdo reading
template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t& value);
template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t& value);
template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t& value);
template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t& value);
template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t& value);
template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint16_t& value);
template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint32_t& value);
template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint64_t& value);
template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, float& value);
template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, double& value);
template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, std::string& value);

// template specialization sdo verivy write
template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t value, float delay);
template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t value, float delay);
template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t value, float delay);
template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t value, float delay);
template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t value, float delay);
template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint16_t value, float delay);
template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint32_t value, float delay);
template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint64_t value, float delay);
template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, float value, float delay);
template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, double value, float delay);

} // namespace ecat_master
