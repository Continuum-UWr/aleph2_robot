#include <vector>

#include "dictionary_error.h"

#include "nanotec_driver/nanotec.h"
#include "nanotec_driver/utils.h"

const static std::unordered_map<uint32_t, std::string> NANOTEC_ERROR_MAP = {
    {0, "Watchdog-Reset"},
    {1, "Input voltage too high"},
    {2, "Output current too high"},
    {3, "Input voltage too low"},
    {4, "Error at fieldbus"},
    {5, "Motor turns – in spite of active block – in the wrong direction"},
    {6, "NMT master takes too long to send nodeguarding request"},
    {7, "Encoder error due to electrical fault or defective hardware"},
    {8, "Encoder error; index not found during the auto setup"},
    {9, "Error in the AB track"},
    {10, "Positive limit switch and tolerance zone exceeded"},
    {11, "Negative limit switch and tolerance zone exceeded"},
    {12, "Device temperature above 80°C"},
    {13, "The values of object 6065h (Following Error Window) and object 6066h Following "
         "Error Time Out) were exceeded; a fault was triggered."},
    {14, "Nonvolatile memory full; controller must be restarted for cleanup work."},
    {15, "Motor blocked"},
    {16, "Nonvolatile memory damaged; controller must be restarted for cleanup work."},
    {17, "Slave took too long to send PDO messages."},
    {18, "Hall sensor faulty"},
    {19, "PDO not processed due to a length error"},
    {20, "PDO length exceeded"},
    {21, "Nonvolatile memory full; controller must be restarted for cleanup work."},
    {22, "Rated current must be set (203Bh:01h)"},
    {23, "Encoder resolution, number of pole pairs and some other values are incorrect."},
    {24, "Motor current is too high, adjust the PI parameters."},
    {25, "Internal software error, generic"},
    {26, "Current too high at digital output"},
    {27, "Unexpected sync length"},
    {28, "EtherCAT only: The motor was stopped because EtherCAT switched state from OP "
         "to either SafeOP or PreOP without first stopping the motor."}
};

const static std::vector<std::string> AUTOCALIB_PARAMETERS = {
    "Motor drive parameter set/Position loop, proportional gain (closed loop)",
    "Motor drive parameter set/Position loop, integral gain (closed loop)",
    "Motor drive parameter set/Velocity loop, proportional gain (closed loop)",
    "Motor drive parameter set/Velocity loop, integral gain (closed loop)",
    "Motor drive parameter set/Flux current loop, proportional gain (closed loop)",
    "Motor drive parameter set/Flux current loop, integral gain (closed loop)",
    "Motor drive parameter set/Torque current loop, proportional gain (closed loop)",
    "Motor drive parameter set/Torque current loop, integral gain (closed loop)",
    "Motor drive parameter set/Torque current loop, proportional gain (open loop)",
    "Motor drive parameter set/Torque current loop, integral gain (open loop)",
};

Nanotec::Nanotec(kaco::Device &device, OperationMode mode)
    : device_(device), operation_mode_(mode)
{
    bool closed_loop =
        uint32_t(device_.get_entry("Motor drive submode select")) & (1 << 0);
    bool bldc = uint32_t(device_.get_entry("Motor drive submode select")) & (1 << 6);
    //bool hall =
        //int32_t(device_.get_entry("Motor drive sensor display closed loop/commutation"));

    if (!closed_loop || !bldc) // || !hall)
    {
        error_ = "Driver not calibrated!";
        return;
    }

    device_.set_entry("Modes of operation", static_cast<int8_t>(operation_mode_));
    SetPowerMode(PowerMode::OFF);
}

void Nanotec::DowloadPowerState()
{
    uint16_t state = device_.get_entry("Statusword");
    bool disabled;

    do
    {
        disabled = false;

        switch (state & 0b01101111)
        {
        case 0: // Not ready to switch on
            disabled = true;
            break;
        case 0b01000000:
        case 0b01100000:                                      // Switch on disabled
            device_.set_entry("controlword", (uint16_t)0x06); // Goto ready
            [[fallthrough]];
        case 0b00100001: // Ready to switch on
            power_state_ = PowerMode::OFF;
            break;
        case 0b00100011: // Switched on
            power_state_ = PowerMode::PASSIVE_BRAKE;
            break;
        case 0b00100111: // Operational
            power_state_ = PowerMode::ACTIVE;
            break;
        default:
            auto error = CheckError();
            assert(error);
            std::cout << "Nanotec is in error state: " << *error;
        }
    } while (disabled);
}

void Nanotec::LoadParameters(const std::map<std::string, int64_t>& parameters)
{
    for (auto const& entry : parameters)
    {
        try 
        {
            kaco::Type type = device_.get_entry_type(entry.first);
            kaco::Value val = int_to_value_of_type(entry.second, type);
            device_.set_entry(entry.first, val);
        } 
        catch (const kaco::dictionary_error& err)
        {
            std::cerr << err.what();
        }
    }
}

std::map<std::string, int64_t> Nanotec::Autocalib()
{
    std::string ret;

    uint16_t data, new_data;

    SetMotorProtection(2000, 2000, 500);

    device_.set_entry("Controlword", uint16_t(0x6)); // Enable Voltage, ~ Quick Stop
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    data = device_.get_entry("Statusword");
    assert((data >> 9) & 1); // Remote
    assert((data >> 5) & 1); // ~ Quick Stop
    assert((data >> 0) & 1); // Ready to switch on

    device_.set_entry("Modes of operation", int8_t(-2)); // Auto setup
    device_.set_entry("Controlword", uint16_t(0x07)); // + Switched On

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    new_data = device_.get_entry("Statusword");
    assert((data & new_data) == data);
    data = new_data;
    assert((data >> 4) & 1); // Voltage Enabled
    assert((data >> 1) & 1); // Switched On

    device_.set_entry("Controlword", uint16_t(0x0f)); // + Enable Operation

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    new_data = device_.get_entry("Statusword");
    data = new_data;
    assert((data & new_data) == data);
    assert((data >> 2) & 1); // Operation Enabled

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    int8_t monitor = device_.get_entry("Modes of operation display");
    assert(monitor == -2); // Auto setup

    device_.set_entry("Controlword", uint16_t(0x1f)); // + Start Auto setup

    uint16_t state;
    do
    {
        state = device_.get_entry("Statusword");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (!((state >> 12) & 1)); // Auto setup completed

    device_.set_entry("Controlword", uint16_t(0x00));

    std::map<std::string, int64_t> parameters;

    for (const std::string& param : AUTOCALIB_PARAMETERS)
    {
        try 
        {
            kaco::Value val = device_.get_entry(param);
            int64_t entry = static_cast<int64_t>(uint32_t(val));
            parameters[param] = entry;
        }
        catch (const kaco::dictionary_error& err)
        {
            std::cerr << err.what() << std::endl;
        }
    }

    SetPowerMode(PowerMode::PASSIVE_BRAKE);
    device_.set_entry("Modes of operation", static_cast<int8_t>(operation_mode_));

    return parameters;
};

void Nanotec::SetPowerMode(PowerMode mode)
{
    while (true)
    {
        DowloadPowerState();

        if (power_state_ == mode)
            return;

        while (power_state_ < mode)
            PowerStateUp();

        if (power_state_ == mode)
            continue;

        power_state_ = mode;
        if (mode == PowerMode::PASSIVE_BRAKE)
            device_.set_entry("controlword", (uint16_t)0b110);
        else if (mode == PowerMode::OFF)
            device_.set_entry("controlword", (uint16_t)0b0);
        else
            assert(0);
    }
};

void Nanotec::SetVelocityProfile(uint32_t max_acceleration, uint32_t max_deceleration)
{
    device_.set_entry("Profile acceleration", max_acceleration);
    device_.set_entry("Profile deceleration", max_deceleration);
}

void Nanotec::SetMotorProtection(uint32_t nominal_current, uint32_t peak_current,
                                 uint32_t peak_length)
{
    device_.set_entry("Maximum current", peak_current);
    device_.set_entry("I2t Parameters/Nominal Current", nominal_current);
    device_.set_entry("I2t Parameters/Maximum duration of peak current", peak_length);
}

void Nanotec::SetTarget(int32_t target)
{
    switch (operation_mode_)
    {
    case OperationMode::TORQUE:
        device_.set_entry("Target torque", static_cast<int16_t>(target));
        break;
    case OperationMode::VELOCITY:
        device_.set_entry("Target velocity", target);
        break;
    case OperationMode::POSITION:
        // device_.set_entry("Polarity", target < 0 ? uint8_t(0b11000000) :
        // uint8_t(0));
        device_.set_entry("Target position", target);
        device_.execute("set_controlword_flag", "controlword_pp_new_set_point");
        device_.execute("unset_controlword_flag", "controlword_pp_new_set_point");
        break;
    }
}

int32_t Nanotec::GetPosition() { return device_.get_entry("Position actual value"); }

int32_t Nanotec::GetVelocity() { return device_.get_entry("Velocity actual value"); }

int16_t Nanotec::GetTorque() { return device_.get_entry("Torque actual value"); }

boost::optional<std::string> Nanotec::CheckError()
{
    std::string ret;
    uint8_t errors_no = device_.get_entry("Pre-defined error field/number of errors");
    if (!errors_no)
        return boost::none;

    for (int i = 1; i <= errors_no; i++)
    {
        uint32_t error_numer = device_.get_entry(0x1003, i);
        ret += NANOTEC_ERROR_MAP.find(error_numer >> 24)->second + ";";
    }

    return ret;
};

void Nanotec::PowerStateUp()
{
    switch (power_state_)
    {
    case PowerMode::OFF:
        device_.set_entry("controlword", (uint16_t)0x07);
        break;
    case PowerMode::PASSIVE_BRAKE:
        device_.set_entry("controlword", (uint16_t)0x0f);
        break;
    case PowerMode::ACTIVE:
        assert(0);
    }

    power_state_ += 1;
};