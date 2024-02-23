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

#include <string>

namespace ecat_master{

/*!
 * Configuration struct for the EthercatMaster
 */
struct EthercatMasterConfiguration{

  /*!
   * Name your buses with a descriptive name. E.g MyArmBus and MyLegsBus if you use multiple buses.
   */
  std::string name{""};

  /*!
   * Network interface name.
   * Use `ip link show` to list available interfaces.
   */
  std::string networkInterface{""};

  /// Communication update time step.
  double timeStep{0.0};

  /*!
   * Threshold before printing a warning about a lowered update rate.
   * This has only an effect if the update is done in one of the standalone modes.
   * - StandaloneEnforceRate update mode:
   *   A warning is printed if update delays could not be compensated during
   *   updateRateTooLowWarnThreshold update cycles.
   * - StandaloneEnforceStep:
   *   A warning is printed if updateRateTooLowWarnThreshold consecutive updates
   *   took too long.
   */

  /*!checks if the pdo size read from the slaves, matches the hardcoded one in the slave sdks.
   * if dynamic PDO mapping is used this check will fail and can therefore be disabled. e.g. Maxon
   */
  bool pdoSizeCheck{false};

  /*!max retires during slave discover on the bus (waiting 1 sec after every try). some slaves might take a bit till started..
   *can be interrupted if startupAbortFlag is nicley wired thru everything.
   */
  unsigned int slaveDiscoverRetries{10};

  unsigned int updateRateTooLowWarnThreshold{50};

  /*!
   * Lower bound to the update step.
   * Even if rate compensation is turned on, the update step will never be
   * lower than rateCompensationCoefficient * timeStep
   * TODO: needs to be tested further
   */
  double rateCompensationCoefficient{0.5};

  /*!
   * Bus diagnosis, reads the bus state, after a hardcoded amount of PDO update cycles, prints enhanced logs if the Bus is not in OPERATIONAL
   */
  bool doBusDiagnosis{false};


  /*!
   * does more bus diagnosis, reads out some error counters after a hardcoded amount of PDO cycles, and logs them to a file found in ~/.ethercat_master/network_interface_name/<datetime>.log
   */
  bool logErrorCounters{false};

};

} // namespace ecat_master
