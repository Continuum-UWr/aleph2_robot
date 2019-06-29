#include <chrono>
#include <cstdint>
#include <thread>

#include "nanotec.h"

int main(int argc, char **argv)
{
    const uint8_t node_id = 1;
    const std::string busname = "can0";
    const std::string baudrate = "500K";

    kaco::Master master;
    if (!master.start(busname, baudrate))
    {
        return EXIT_FAILURE;
    }

    bool found_device = false;
    size_t device_index;

    while (!found_device)
    {

        for (size_t i = 0; i < master.num_devices(); ++i)
        {
            kaco::Device &device = master.get_device(i);
            if (device.get_node_id() == node_id)
            {
                found_device = true;
                device_index = i;
                break;
            }
        }

        std::cout
            << "Device with ID " << (unsigned)node_id
            << " has not been found yet. Will keep retrying. Press Ctrl+C to abort."
            << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    kaco::Device &device = master.get_device(device_index);

    device.start();
    device.load_dictionary_from_library();

    Nanotec nanotec(device, Nanotec::OperationMode::VELOCITY);

    std::cout << "Calibration results " << nanotec.Autocalib();

}
