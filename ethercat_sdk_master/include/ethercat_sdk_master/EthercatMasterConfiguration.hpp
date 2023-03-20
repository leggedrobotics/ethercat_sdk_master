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
  unsigned int updateRateTooLowWarnThreshold{50};

  /*!
   * Lower bound to the update step.
   * Even if rate compensation is turned on, the update step will never be
   * lower than rateCompensationCoefficient * timeStep
   * TODO: needs to be tested further
   */
  double rateCompensationCoefficient{0.5};
};

} // namespace ecat_master
