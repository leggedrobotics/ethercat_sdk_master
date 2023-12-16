#include "ethercat_sdk_master/EthercatMasterSingleton.hpp"

namespace ecat_master {
    EthercatMasterSingleton&  EthercatMasterSingleton::instance() {
        static EthercatMasterSingleton instance_;
        return instance_;
    }
}