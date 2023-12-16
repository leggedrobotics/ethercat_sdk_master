#pragma once

#include <ethercat_sdk_master/EthercatMaster.hpp>
#include <map>

namespace ecat_master {
    /**
     *  @brief Provides the only method how we can use the same ethercat bus in multiple ros2control hardware interfaces
     * The idea is that we centrally manage the instances of the EthercatMasters and each hardware interface may attach its devices to it
    */
    class EthercatMasterSingleton {
        public:
            static EthercatMasterSingleton&  instance(); //Method has to be in cpp file in order to work properly

            std::shared_ptr<EthercatMaster> get(const EthercatMasterConfiguration& config) {
                std::lock_guard<std::recursive_mutex> guard(lock_);

                if(ecat_masters_.find(config.networkInterface) == ecat_masters_.end())
                {
                    MELO_INFO_STREAM("Setting up new EthercatMaster on interface: " << config.networkInterface << " and updating it");
                    auto master = std::make_shared<EthercatMaster>();
                    master->loadEthercatMasterConfiguration(config);
                    
                    //Spin the master asynchronously
                    spin_threads_.emplace(config.networkInterface, std::make_unique<std::thread>(std::bind(&EthercatMasterSingleton::spin,this, std::placeholders::_1), master));
                }
                
                if(config != ecat_masters_[config.networkInterface]->getConfiguration()){
                    //Print warning or abort if the configuration does not match!
                    MELO_WARN_STREAM("Ethercat master configurations do not match for bus: " << config.networkInterface);
                }

                return ecat_masters_[config.networkInterface];
            }

            bool hasMaster(const EthercatMasterConfiguration& config){
                return ecat_masters_.find(config.networkInterface) != ecat_masters_.end();
            }
            bool hasMaster(const std::string& networkInterface) {
                return ecat_masters_.find(networkInterface) != ecat_masters_.end();
            }

            std::shared_ptr<EthercatMaster> operator[] (const EthercatMasterConfiguration& config){
                return get(config);
            }
        private:

        EthercatMasterSingleton(){

        }
        ~EthercatMasterSingleton() {
            abort_ = true;

            for(const auto& [interface, thread]: spin_threads_){
                thread->join();
            }
            for(const auto &  [interfrace, master]: ecat_masters_){
                master->preShutdown();
                master->shutdown();
            }
        }
        void spin(std::shared_ptr<EthercatMaster> master_){
            master_->setRealtimePriority(); 
            while(!abort_){
                master_->update(UpdateMode::StandaloneEnforceRate);
            }
        }

        std::map<std::string,std::shared_ptr<EthercatMaster>> ecat_masters_;
        std::map<std::string, std::unique_ptr<std::thread>> spin_threads_;
        std::recursive_mutex lock_;
        std::atomic_bool abort_ = false;
    };

}