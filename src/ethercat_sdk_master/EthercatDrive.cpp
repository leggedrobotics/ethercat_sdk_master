#include "ethercat_sdk_master/EthercatDrive.hpp"

#include <iostream>

namespace ecat_master{

// Sdo write template specialization
template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int8_t value) {
  bool success = sendSdoWriteInt8(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: int8_t" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int16_t value) {
  bool success = sendSdoWriteInt16(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: int16_t" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int32_t value) {
  bool success = sendSdoWriteInt32(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: int32_t" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int64_t value) {
  bool success = sendSdoWriteInt64(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: int64_t" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint8_t value) {
  bool success = sendSdoWriteUInt8(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: uint8_t" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint16_t value) {
  bool success = sendSdoWriteUInt16(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: uint16_t" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint32_t value) {
  bool success = sendSdoWriteUInt32(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: uint32_t" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint64_t value) {
  bool success = sendSdoWriteUInt64(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: uint64_t" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const float value) {
  bool success = sendSdoWriteFloat(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: float" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const double value) {
  bool success = sendSdoWriteDouble(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: double" << std::endl;
  }
  return success;
}

// Sdo read specialization
template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t& value) {
  bool success = sendSdoReadInt8(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: int8_t" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t& value) {
  bool success = sendSdoReadInt16(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: int16_t" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t& value) {
  bool success = sendSdoReadInt32(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: int32_t" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t& value) {
  bool success = sendSdoReadInt64(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: int64_t" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t& value) {
  bool success = sendSdoReadUInt8(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: uint8_t" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint16_t& value) {
  bool success = sendSdoReadUInt16(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: uint16_t" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint32_t& value) {
  bool success = sendSdoReadUInt32(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: uint32_t" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint64_t& value) {
  bool success = sendSdoReadUInt64(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: uint64_t" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, float& value) {
  bool success = sendSdoReadFloat(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: float" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, double& value) {
  bool success = sendSdoReadDouble(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: double" << std::endl;
  }
  return success;
}

template <>
bool EthercatDrive::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, std::string& value) {
  bool success = sendSdoReadString(index, subindex, completeAccess, value);
  if (!success) {
    std::cout << "Error writing SDO.\nIndex: " << (int)index << "\nSubindex: " << (int)subindex << "\n Complete Access: " << (int)completeAccess << "\nType: std::string" << std::endl;
  }
  return success;
}

// Sdo Verify write
template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t value, float delay) {
  int8_t testVal;
  sendSdoWriteInt8(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadInt8(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t value, float delay) {
  int16_t testVal;
  sendSdoWriteInt16(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadInt16(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t value, float delay) {
  int32_t testVal;
  sendSdoWriteInt32(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadInt32(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t value, float delay) {
  int64_t testVal;
  sendSdoWriteInt64(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadInt64(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t value, float delay) {
  uint8_t testVal;
  sendSdoWriteUInt8(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadUInt8(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint16_t value, float delay) {
  uint16_t testVal;
  sendSdoWriteUInt16(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadUInt16(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint32_t value, float delay) {
  uint32_t testVal;
  sendSdoWriteUInt32(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadUInt32(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint64_t value, float delay) {
  uint64_t testVal;
  sendSdoWriteUInt64(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadUInt64(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, float value, float delay) {
  float testVal;
  sendSdoWriteFloat(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadFloat(index, subindex, completeAccess, testVal);
  return (value == testVal);
}

template <>
bool EthercatDrive::sdoVerifyWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, double value, float delay) {
  double testVal;
  sendSdoWriteDouble(index, subindex, completeAccess, value);
  usleep(static_cast<unsigned int>(delay));
  sendSdoReadDouble(index, subindex, completeAccess, testVal);
  return (value == testVal);
}
} // namespace ecat_master
