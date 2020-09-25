#pragma once

#include <string>

namespace ecat_master{

struct EthercatMasterConfiguration{

  std::string networkInterface{""};
  double timeStep{0.0};
  unsigned int ethercatStateChangeTimeout{10000}; // microseconds
  unsigned int updateRateTooLowWarnThreshold{50};

};

} // namespace ecat_master
