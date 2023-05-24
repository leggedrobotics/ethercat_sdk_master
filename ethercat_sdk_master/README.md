# ethercat_sdk_master

The 'ethercat_sdk_master' is a common master implementation that can be used to interface multiple ethercat slaves on the same bus. 
An example how this package can be found in the 'elmo_ethercat_sdk'

# Dependencies

## RSL packages

| Repo           | url                                                  | License      | Content                                          |
|:--------------:|:----------------------------------------------------:|:------------:|:------------------------------------------------:|
| soem_interface_rsl | https://github.com/leggedrobotics/soem_interface_rsl.git | GPLv3        | EtherCAT functionalities                         |
| message_logger | https://github.com/leggedrobotics/message_logger.git | BSD 3-Clause | simple log streams                               |

## System Dependencies (Ubuntu 18.04 LTS)

- [ROS Melodic](https://wiki.ros.org/melodic)
- catkin
- yaml-cpp
- gcc > 8.0
