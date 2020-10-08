
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


#include "ethercat_sdk_master/EthercatDevice.hpp"

#include <iostream>

namespace ecat_master{

void EthercatDevice::setTimeStep(double timeStep){
  timeStep_ = timeStep;
}

// Sdo write template specialization
template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int8_t value) {
  bool success = sendSdoWriteInt8(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: int8_t");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int16_t value) {
  bool success = sendSdoWriteInt16(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: int16_t");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int32_t value) {
  bool success = sendSdoWriteInt32(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: int32_t");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int64_t value) {
  bool success = sendSdoWriteInt64(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: int64_t");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint8_t value) {
  bool success = sendSdoWriteUInt8(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: uint8_t");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint16_t value) {
  bool success = sendSdoWriteUInt16(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: uint16_t");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint32_t value) {
  bool success = sendSdoWriteUInt32(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: uint32_t");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint64_t value) {
  bool success = sendSdoWriteUInt64(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: uint64_t");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const float value) {
  bool success = sendSdoWriteFloat(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: float");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const double value) {
  bool success = sendSdoWriteDouble(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: double");
  }
  return success;
}

// Sdo read specialization
template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t& value) {
  bool success = sendSdoReadInt8(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: int8_t");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t& value) {
  bool success = sendSdoReadInt16(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: int16_t");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t& value) {
  bool success = sendSdoReadInt32(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: int32_t");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t& value) {
  bool success = sendSdoReadInt64(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: int64_t");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t& value) {
  bool success = sendSdoReadUInt8(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: uint8_t");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint16_t& value) {
  bool success = sendSdoReadUInt16(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: uint16_t");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint32_t& value) {
  bool success = sendSdoReadUInt32(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: uint32_t");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint64_t& value) {
  bool success = sendSdoReadUInt64(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: uint64_t");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, float& value) {
  bool success = sendSdoReadFloat(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: float");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, double& value) {
  bool success = sendSdoReadDouble(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: double");
  }
  return success;
}

template <>
bool EthercatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, std::string& value) {
  bool success = sendSdoReadString(index, subindex, completeAccess, value);
  if (!success) {
    MELO_ERROR_STREAM("Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: std::string");
  }
  return success;
}

// Sdo Verify write
template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t value, float delay) {
  int8_t testVal;
  sendSdoWriteInt8(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadInt8(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t value, float delay) {
  int16_t testVal;
  sendSdoWriteInt16(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadInt16(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t value, float delay) {
  int32_t testVal;
  sendSdoWriteInt32(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadInt32(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t value, float delay) {
  int64_t testVal;
  sendSdoWriteInt64(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadInt64(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t value, float delay) {
  uint8_t testVal;
  sendSdoWriteUInt8(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadUInt8(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint16_t value, float delay) {
  uint16_t testVal;
  sendSdoWriteUInt16(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadUInt16(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint32_t value, float delay) {
  uint32_t testVal;
  sendSdoWriteUInt32(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadUInt32(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint64_t value, float delay) {
  uint64_t testVal;
  sendSdoWriteUInt64(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadUInt64(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, float value, float delay) {
  float testVal;
  sendSdoWriteFloat(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadFloat(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDevice::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, double value, float delay) {
  double testVal;
  sendSdoWriteDouble(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadDouble(index, subindex, completeAccess, testVal);
  return (value == testVal);
}
} // namespace ecat_master
