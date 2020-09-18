#pragma once

#include <string>

namespace ecat_master{

struct EthercatMasterConfiguration{

  std::string networkInterface{""};
  double timeStep{0.0};
  unsigned int ethercatStateChangeTimeout{10000}; // microseconds

};

} // namespace ecat_master
