#include "ethercat_sdk_master/EthercatDevice.hpp"
#include "ethercat_sdk_master/EthercatMasterConfiguration.hpp"
#include "ethercat_sdk_master/UpdateMode.hpp"

#include <soem_interface/EthercatBusBase.hpp>

#include <memory>
#include <vector>
#include <chrono>

namespace ecat_master{

/*!
 * Ethercat Master Class.
 * Offers a simple API to:
 * - Attach devices of type EthercatDevice that shall be managed.
 * - Startup the EtherCAT communication and configure the attached devices
 * - Update the communication at a constant rate if desired
 * - Shutdown the communication
 */
class EthercatMaster {
public:
  typedef std::shared_ptr<EthercatMaster> SharedPtr;
public:
  EthercatMaster() = default;

  /*!
   * Initialize the soem_interface::EthercatBusBase bus_ object.
   */
  void createEthercatBus();

  /*!
   * Attach an EtherCAT device.
   * @param[in] device std::shared_ptr to object derive from ecat_master::EthercatDevice
   * @return true if a device of the given name does not yet exist
   */
  bool attachDevice(EthercatDevice::SharedPtr device);

  /*!
   * Start the EtherCAT communication.
   * The startup() method of each attached EtherCAT device is called.
   * The devices are then set into the OPERATIONAL EtherCAT state.
   * @return true if successful.
   */
  bool startup();

  /*!
   * Update the PDO communication.
   * Propagate the EtherCAT frame(s) through the network.
   * This function needs to be called at a constant rate.
   * There are three options:
   *   1. use updateMode = UpdateMode::NonStandalone if the timing is handled externally.
   *   2. use updateMode = UpdateMode::StandaloneEnforceRate if the call shall
   *      create the necessary timeout such that the average update rate corresponds
   *      to the configured timestep.
   *   3. use updateMode == UpdateMode::StandaloneEnforceStep if the call shall
   *      create the necessary timeout such that the update time step corresponds
   *      to the configured time step.
   * @param[in] updateMode select the update mode.
   */
  void update(UpdateMode updateMode);

  /*!
   * Shutdown the communication.
   * EtherCAT communication is not possible after calling shutdown.
   */
  void shutdown();

  /*!
   * Pre shutdown communication.
   * Call preShutdown() of every attached device.
   * This function needs to be executed outside of the communicatino update thread.
   * PDO communication needs to continue until this funtion returns.
   * @see https://bitbucket.org/leggedrobotics/ethercat_device_configurator/src/master/src/standalone.cpp
   */
  void preShutdown();

  /*!
   * Returns a raw pointer to the bus_ object.
   */
  soem_interface::EthercatBusBase* getBusPtr() { return bus_.get(); }


// Configuration
public:

  /*!
   * Load a configuration.
   * @param[in] configuration EthercatMasterConfiguration.
   */
  void loadEthercatMasterConfiguration(const EthercatMasterConfiguration& configuration);

  /*!
   * Return the active configuration.
   * @return configuration_
   */
  EthercatMasterConfiguration getConfiguration();

protected:
  std::unique_ptr<soem_interface::EthercatBusBase> bus_{nullptr};
  std::vector<EthercatDevice::SharedPtr> devices_;
  EthercatMasterConfiguration configuration_;
  std::chrono::time_point<std::chrono::high_resolution_clock> lastWakeupTime_;
  std::chrono::high_resolution_clock::duration updateDuration_;
  std::chrono::high_resolution_clock::duration deltaT_;
  std::chrono::high_resolution_clock::duration targetUpdateDuration_;
  unsigned int rateTooLowCounter_{0};

protected:
  bool deviceExists(const std::string& name);
  void syncDistributedClock0(const std::vector<uint32_t>& addresses);

  /*!
   * Let the update thread sleep such that the desired update rate is created.
   * - If enforceRate is true:
   *   Compensate for update cycles that took to long by sleeping for a shorter time period.
   *   The average rate will correspond to the desired rate (if possible).
   * - If enforceRate is false:
   *   Do not compensate for update cycles that took too long.
   *   Every timestep is kept as close to the desired value as possible.
   */
  void createUpdateHeartbeat(bool enforceRate);


};
} // namespace ecat_master
