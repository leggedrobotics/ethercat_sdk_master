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
};

} // namespace ecat_master
