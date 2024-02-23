/*
 ** Copyright 2020 Robotic Systems Lab - ETH Zurich:
 ** Lennart Nachtigall, Jonas Junger
 ** Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions
 *are met:
 **
 ** 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 **
 ** 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
 *documentation and/or other materials provided with the distribution.
 **
 ** 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from
 *this software without specific prior written permission.
 **
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ethercat_sdk_master/EthercatMaster.hpp"
#include <pthread.h>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <thread>
#include "message_logger/message_logger.hpp"

#define SLEEP_EARLY_STOP_NS (50000)  // less than 1e9 - 1!!
#define BILLION (1000000000)

namespace ecat_master {

void EthercatMaster::loadEthercatMasterConfiguration(const EthercatMasterConfiguration& configuration) {
  using namespace std::chrono_literals;
  devices_.clear();
  configuration_ = configuration;

  timestepNs_ = floor(configuration.timeStep * 1e9);
  if (configuration_.logErrorCounters) {
    auto getFolderSize = [](const std::string& folderPath) -> uintmax_t {
      std::uintmax_t totalSize = 0;

      for (const auto& entry : std::filesystem::recursive_directory_iterator(folderPath)) {
        if (std::filesystem::is_regular_file(entry)) {
          totalSize += std::filesystem::file_size(entry);
        }
      }
      return totalSize;
    };
    std::string logFolderName{std::string{std::getenv("HOME")} + "/.ethercat_master/" + configuration_.networkInterface};
    if (!std::filesystem::exists(logFolderName)) {
      if (std::filesystem::create_directories(logFolderName)) {
        MELO_INFO_STREAM("[EthercatMaster::" << configuration_.networkInterface << "] Created logging dir: " << logFolderName);
      }
    }
    auto folderSize = getFolderSize(logFolderName);
    MELO_DEBUG_STREAM("[Ethercatmaster::" << configuration_.networkInterface << "] Log file in folder ~/.ethercat_master/"
                                          << configuration_.networkInterface << ", Foldersize: " << folderSize)
    if (folderSize > static_cast<uintmax_t>(500 * 10e6)) {
      MELO_ERROR_STREAM("[Ethercatmaster::" << configuration_.networkInterface << "] Log file in folder ~/.ethercat_master/"
                                            << configuration_.networkInterface
                                            << " is larger than 500Mb. consider cleaning up! Foldersize: " << folderSize)
    }

    auto currentTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&currentTime), "%Y-%m-%d_%H:%M:%S");

    std::string logFilename = logFolderName + "/" + ss.str() + ".log";

    {
      std::lock_guard busDiagStreamLock(logFileStreamMutex_);
      busDiagnosisLogFile_ = std::fstream(logFilename, std::ios::out);
      if (busDiagnosisLogFile_.is_open()) {
        MELO_INFO_STREAM("[Ethercatmaster::" << configuration_.networkInterface << "] Writing to logfile: " << logFilename)
      }
    }
  }
  createEthercatBus();
}

EthercatMasterConfiguration EthercatMaster::getConfiguration() {
  return configuration_;
}

void EthercatMaster::createEthercatBus() {
  bus_.reset(new soem_interface_rsl::EthercatBusBase(configuration_.networkInterface));
}

bool EthercatMaster::attachDevice(EthercatDevice::SharedPtr device) {
  if (deviceExists(device->getName())) {
    MELO_ERROR_STREAM("Cannot attach device with name '" << device->getName() << "' because it already exists.");
    return false;
  }
  bus_->addSlave(device);
  device->setEthercatBusBasePointer(bus_.get());
  device->setTimeStep(configuration_.timeStep);
  devices_.push_back(device);
  MELO_DEBUG_STREAM("Attached device '" << device->getName() << "' to address " << device->getAddress());
  return true;
}

bool EthercatMaster::startup(std::atomic<bool>& abortFlag) {
  bool success = true;

  success &= bus_->startup(abortFlag, configuration_.pdoSizeCheck, configuration_.slaveDiscoverRetries);
  if (!success) {
    return false;
  }

  for (const auto& device : devices_) {
    if (!bus_->waitForState(soem_interface_rsl::ETHERCAT_SM_STATE::SAFE_OP, device->getAddress(), 50)) {
      MELO_ERROR_STREAM("[EthercatMaster::" << bus_->getName() << "] not in SAFE_OP after startup!");
    }
  }

  // write the header of the diagnosis log
  if (configuration_.logErrorCounters) {
    if (busDiagnosisLogFile_) {
      std::lock_guard busDiagStreamLock(logFileStreamMutex_);
      busDiagnosisLogFile_ << "Time, " << configuration_.networkInterface << ", ";
      for (size_t slaveCount = 0; slaveCount < devices_.size(); slaveCount++) {
        for (size_t regCount = 0; regCount < static_cast<size_t>(soem_interface_rsl::REG::ERROR_COUNTERS::SIZE); regCount++) {
          busDiagnosisLogFile_ << devices_[slaveCount]->getName();  // For every error register a column.
          bool lastElement =
              (slaveCount == devices_.size() - 1) && (regCount == static_cast<size_t>(soem_interface_rsl::REG::ERROR_COUNTERS::SIZE) - 1);
          if (!lastElement) {
            busDiagnosisLogFile_ << ", ";
          }
        }
      }

      logStartTime_ = std::chrono::system_clock::now();
      auto currentTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
      std::stringstream ss;
      ss << std::put_time(std::localtime(&currentTime), "%Y-%m-%d_%H:%M:%S");

      busDiagnosisLogFile_ << "\n" << ss.str();
      busDiagnosisLogFile_ << ", ALStatusCode, ";  // DLStatus
      for (size_t slaveCount = 0; slaveCount < devices_.size(); slaveCount++) {
        for (size_t regCount = 0; regCount < static_cast<size_t>(soem_interface_rsl::REG::ERROR_COUNTERS::SIZE); regCount++) {
          busDiagnosisLogFile_ << soem_interface_rsl::REG::ERROR_COUNTERS_LIST.Registers[regCount].name;
          bool lastElement =
              (slaveCount == devices_.size() - 1) && (regCount == static_cast<size_t>(soem_interface_rsl::REG::ERROR_COUNTERS::SIZE) - 1);
          if (!lastElement) {
            busDiagnosisLogFile_ << ", ";
          }
        }
      }
      busDiagnosisLogFile_ << std::endl;  // this flushes.
      busDiagnosisLog_.errorCounters_.resize(devices_.size());
    } else {
      MELO_ERROR_STREAM("[EthercatMaster::" << bus_->getName() << "] Could not open/write log file.")
    }
  }

  if (!success) MELO_ERROR("[ethercat_sdk_master:EthercatMaster::startup] Startup not successful.");
  return success;
}

bool EthercatMaster::startup() {
  std::atomic<bool> tmpFlag{false};
  return startup(tmpFlag);
}

bool EthercatMaster::activate() {
  bool success = true;

  bus_->setState(soem_interface_rsl::ETHERCAT_SM_STATE::OPERATIONAL);
  success &= bus_->waitForState(soem_interface_rsl::ETHERCAT_SM_STATE::OPERATIONAL, 0, 0);

  // will only be used in case internal update timing functionality is used, otherwise no effect.
  firstUpdate_ = true;
  return success;
}

bool EthercatMaster::deactivate() {
  // is there any action on the slaves needed?, the slaves EC SM and Drive SM should take care of it?

  bool success = true;
  bus_->setState(soem_interface_rsl::ETHERCAT_SM_STATE::SAFE_OP);
  success &= bus_->waitForState(soem_interface_rsl::ETHERCAT_SM_STATE::SAFE_OP, 0, 0);
  return success;
}

void EthercatMaster::update(UpdateMode updateMode) {
  if (firstUpdate_) {
    clock_gettime(CLOCK_MONOTONIC, &lastWakeup_);
    sleepEnd_ = lastWakeup_;
    firstUpdate_ = false;
  }
  bus_->updateWrite();
  bus_->updateRead();

  // log
  if (configuration_.doBusDiagnosis) {
    if (busDiagDecimationCount_ >
        200) {  // after 100 pdo cycles a 1 diagnosis datagram send, this datagram swaps between error counter or state depending on config.
      bus_->doBusMonitoring(configuration_.logErrorCounters);
      if (configuration_.logErrorCounters) {
        bool diagUpdated = bus_->getBusDiagnosisLog(busDiagnosisLog_);
        if (diagUpdated) {  // will only be fully after some runs, depends on number of slaves on the bus.
          MELO_DEBUG_STREAM("[EcatMaster::" << bus_->getName() << "::Update] Writing log to file (or buffer)")
          // write the error counter to the file:
          auto currentTime = std::chrono::system_clock::now();
          auto msSinceStart =
              std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch() - logStartTime_.time_since_epoch());
          std::chrono::seconds secondsSinceStart = std::chrono::duration_cast<std::chrono::seconds>(msSinceStart);
          std::chrono::milliseconds millisecondsSinceStart =
              std::chrono::duration_cast<std::chrono::milliseconds>(msSinceStart % std::chrono::seconds(1));
          std::lock_guard busDiagStreamLock(logFileStreamMutex_);
          busDiagnosisLogFile_ << secondsSinceStart.count() << "." << std::setw(3) << std::setfill('0') << millisecondsSinceStart.count()
                               << ", ";
          busDiagnosisLogFile_ << busDiagnosisLog_.ecatApplicationLayerStatus << ", ";
          for (size_t slaveCount = 0; slaveCount < busDiagnosisLog_.errorCounters_.size(); slaveCount++) {
            for (size_t errorRegCount = 0; errorRegCount < static_cast<size_t>(soem_interface_rsl::REG::ERROR_COUNTERS::SIZE);
                 errorRegCount++) {
              busDiagnosisLogFile_ << busDiagnosisLog_.errorCounters_[slaveCount][errorRegCount].fullValue;
              bool lastEntry = (slaveCount == busDiagnosisLog_.errorCounters_.size() - 1) &&
                               (errorRegCount == static_cast<size_t>(soem_interface_rsl::REG::ERROR_COUNTERS::SIZE) - 1);
              if (!lastEntry) {
                busDiagnosisLogFile_ << ", ";
              }
            }
          }
          busDiagnosisLogFile_ << std::endl;  // flush after every loop.
        }
      }
      busDiagDecimationCount_ = 0;
    }
    busDiagDecimationCount_++;
  }
  // we should flush here to not leave the function (and therefore the thread

  // create update heartbeat if in standalone mode
  switch (updateMode) {
    case UpdateMode::StandaloneEnforceRate:
      createUpdateHeartbeat(true);
      break;
    case UpdateMode::StandaloneEnforceStep:
      createUpdateHeartbeat(false);
      break;
    case UpdateMode::NonStandalone:
      break;
  }
}

void EthercatMaster::shutdown() {
  if (bus_) {
    bus_->setState(soem_interface_rsl::ETHERCAT_SM_STATE::INIT);
    bus_->shutdown();
  }
  bus_.reset(nullptr);
}

void EthercatMaster::preShutdown(bool setIntoSafeOP) {
  if (bus_) {  // check if the bus is not shutdown already..
    for (auto& device : devices_) {
      if (device) {
        device->preShutdown();
        MELO_DEBUG_STREAM("Test test")
      }
    }

    MELO_DEBUG_STREAM("Test test test")

    if (setIntoSafeOP) {
      MELO_DEBUG_STREAM("[EthercatMaster::" << bus_->getName() << "] Trying to deavtivete the bus")
      // immediately fall back to SAFE_OP so that no PDO timeout triggered during shutdown. PDO readings will still be received, slave
      // outputs are active but in "safe" state. probably vendor dependent what safe state means. after preShutdown slave should be in a
      // state which allows to fallback into EC_STATE_SAFE_OP without triggering any further slave Call
      bus_->setState(soem_interface_rsl::ETHERCAT_SM_STATE::SAFE_OP);
      bus_->waitForState(soem_interface_rsl::ETHERCAT_SM_STATE::SAFE_OP);
    }
  }
}

EthercatMaster::~EthercatMaster() {
  // make sure that the bus is shutdown.
  shutdown();
  std::lock_guard busDiagStreamLock(logFileStreamMutex_);
  if (busDiagnosisLogFile_.is_open()) {
    busDiagnosisLogFile_.flush();
    busDiagnosisLogFile_.close();
  }
}

bool EthercatMaster::deviceExists(const std::string& name) {
  for (const auto& device : devices_) {
    if (device->getName() == name) {
      return true;
    }
  }
  return false;
}

bool EthercatMaster::setRealtimePriority(int priority, int cpu_core) const {
  bool success = true;
  // Handle to our thread
  pthread_t thread = pthread_self();
  // First set the priority of the thread in the scheduler to the passed priority (99 is max)
  sched_param param;
  param.sched_priority = priority;
  int errorFlag = pthread_setschedparam(thread, SCHED_FIFO, &param);
  if (errorFlag != 0) {
    MELO_ERROR_STREAM("[ethercat_sdk_master:EthercatMaster::setRealtimePriority]"
                      << " Could not set thread priority. Check limits.conf or"
                      << " execute as root");
    success &= false;
  }

  // Allow attaching the thread to a certain cpu core
  // Create an empty cpu set for the scheduler
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  // Obtain amount of cpus
  int number_of_cpus = sysconf(_SC_NPROCESSORS_ONLN);

  // In case the user passed a core value > 0
  if (cpu_core > 0) {
    // check if the core is < than the number of available cpus
    if (cpu_core >= number_of_cpus) {
      MELO_ERROR_STREAM("[ethercat_sdk_master:EthercatMaster::setRealtimePriority]"
                        << "Tried to attach thread to core: " << cpu_core << " even though we only have: " << number_of_cpus << " core!");
      return false;
    }
    // Set the core
    CPU_SET(cpu_core, &cpuset);
    // Tell the scheduler our preferences
    errorFlag = pthread_setaffinity_np(thread, sizeof(cpuset), &cpuset);
    if (errorFlag != 0) {
      MELO_ERROR_STREAM("[ethercat_sdk_master:EthercatMaster::setRealtimePriority]"
                        << "Could not assign ethercat thread to single cpu core: " << errorFlag);
      success &= false;
    }
  }
  return success;
}

////////////////////////////
// Timing functionalities //
////////////////////////////

// true if ts1 < ts2
inline bool timespecSmallerThan(timespec* ts1, timespec* ts2) {
  return (ts1->tv_sec < ts2->tv_sec || (ts1->tv_sec == ts2->tv_sec && ts1->tv_nsec < ts2->tv_nsec));
}

inline void highPrecisionSleep(timespec ts) {
  if (ts.tv_nsec >= SLEEP_EARLY_STOP_NS) {
    ts.tv_nsec -= SLEEP_EARLY_STOP_NS;
  } else {
    ts.tv_nsec += BILLION - SLEEP_EARLY_STOP_NS;
    ts.tv_sec -= 1;
  }
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);

  // busy waiting for the remaining time
  timespec now;
  do {
    clock_gettime(CLOCK_MONOTONIC, &now);
  } while (timespecSmallerThan(&now, &ts));
}

inline void addNsecsToTimespec(timespec* ts, long int ns) {
  if (ts->tv_nsec < BILLION - (ns % BILLION)) {
    ts->tv_nsec += ns % BILLION;
    ts->tv_sec += ns / BILLION;
  } else {
    ts->tv_nsec += (ns % BILLION) - BILLION;
    ts->tv_sec += ns / BILLION + 1;
  }
}

inline long int getTimeDiffNs(timespec* t_end, timespec* t_start) {
  return BILLION * (t_end->tv_sec - t_start->tv_sec) + t_end->tv_nsec - t_start->tv_nsec;
}

long EthercatMaster::getUpdateTimeNs() {
  std::lock_guard<std::mutex> lock(timeStepMutex_);
  return timeStepNsMeasured_;
}

void EthercatMaster::createUpdateHeartbeat(bool enforceRate) {
  timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);

  if (enforceRate) {
    // since we do sleep in absolute times keeping the rate constant is trivial:
    // we simply increment the target sleep wakeup time by the timestep in every
    // iteration.
    addNsecsToTimespec(&sleepEnd_, timestepNs_);
  } else {
    // sleep until timeStepNs_ nanoseconds from last wakeup time
    sleepEnd_ = lastWakeup_;
    addNsecsToTimespec(&sleepEnd_, timestepNs_);
  }

  // we are late.
  if (timespecSmallerThan(&sleepEnd_, &now)) {
    rateTooLowCounter_++;
    accumulatedDelayNs_ = accumulatedDelayNs_ + getTimeDiffNs(&now, &sleepEnd_);  // might overflow
    // prevent the creation of a too low update step
    addNsecsToTimespec(&lastWakeup_, static_cast<long int>(configuration_.rateCompensationCoefficient * timestepNs_));
    // we need to sleep a bit
    if (timespecSmallerThan(&now, &lastWakeup_)) {
      highPrecisionSleep(lastWakeup_);
    }

    if (rateTooLowCounter_ >= configuration_.updateRateTooLowWarnThreshold) {
      MELO_DEBUG_STREAM(
          "[ethercat_sdk_master:EthercatMaster::createUpdateHeartbeat]: update rate too low, accumulated delay: " << accumulatedDelayNs_);
    }

  } else {
    rateTooLowCounter_ = 0;
    accumulatedDelayNs_ = 0;
    highPrecisionSleep(sleepEnd_);
  }
  timespec measurementTime;
  clock_gettime(CLOCK_MONOTONIC, &measurementTime);
  {
    std::lock_guard<std::mutex> lock(timeStepMutex_);
    timeStepNsMeasured_ = getTimeDiffNs(&measurementTime, &lastWakeup_);
  }
  clock_gettime(CLOCK_MONOTONIC, &lastWakeup_);
}

}  // namespace ecat_master
