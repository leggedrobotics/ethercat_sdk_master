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

#include "EthercatDevice.hpp"

namespace ecat_master {
class EthercatActuatorDevice: public EthercatDevice
{

public:
    typedef std::shared_ptr<EthercatActuatorDevice> SharedPtr;

    class EthercatActuatorDeviceReading
    {
    public:
        double currentQ = 0;        //A
        double motorVelocity = 0;   //rad/s
        double motorPosition = 0;   //rad
        double temperature = 0;     //°C
        std::chrono::system_clock::time_point timeStamp; //Time
    };

    enum class EthercatActuatorDeviceControlMode
    {
        None = 0,
        Current = 1,
        Velocity = 2,
        Position = 3,
    };

    class EthercatActuatorDeviceCommand
    {
    public:
        EthercatActuatorDeviceControlMode mode = EthercatActuatorDeviceControlMode::None;
        double currentQdes = 0; //A
        double motorVelocityDes = 0; //rad/s
        double motorPositionDes = 0; //rad
    };

    virtual EthercatActuatorDeviceReading getLatestReading() const = 0;
    virtual bool setCommand(const EthercatActuatorDeviceCommand& command) = 0;


};
}
