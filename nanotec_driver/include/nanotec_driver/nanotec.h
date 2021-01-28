
#pragma once

#include <map>

#include "kacanopen/core/canopen_error.h"
#include "kacanopen/core/logger.h"
#include "kacanopen/master/master.h"

class Nanotec
{

  public:
    enum PowerMode
    {
        OFF,
        PASSIVE_BRAKE,
        ACTIVE
    };

    enum OperationMode
    {
        NEUTRAL = 0,
        POSITION = 1,
        VELOCITY = 3,
        TORQUE = 4
    };

    Nanotec(kaco::Device &device, OperationMode mode);

    void LoadParameters(const std::map<std::string, int64_t>& parameters);

    std::map<std::string, int64_t> Autocalib();

    void SetPowerMode(PowerMode mode);
    void SetOperationMode(OperationMode mode);

    void SetVelocityProfile(uint32_t max_acceleration, uint32_t max_deceleration);
    void SetMotorProtection(uint32_t nominal_current, uint32_t peak_current,
                            uint32_t peak_length);
    void SetTarget(int32_t target);

    int32_t GetPosition();
    int32_t GetVelocity();
    int16_t GetTorque();

    boost::optional<std::string> CheckError();

  private:
    kaco::Device &device_;
    uint32_t operation_mode_;
    uint32_t power_state_;
    boost::optional<std::string> error_;

    void PowerStateUp();
    void DowloadPowerState();
};