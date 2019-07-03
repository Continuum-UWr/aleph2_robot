#include <chrono>
#include <cstdint>
#include <thread>
#include <unistd.h>

#include "nanotec_driver/nanotec.h"

int main(int argc, char **argv)
{
    uint8_t node_id = 1;
    std::string busname = "can0";
    std::string baudrate = "500K";

    int opt;
    while((opt = getopt(argc, argv, "c:n:b:")) != -1) 
    {
        switch(opt)
        {
            case 'c':
                busname = optarg;
                break;
            case 'n':
                node_id = static_cast<uint8_t>(std::stoi(optarg));
                break;
            case 'b':
                baudrate = optarg;
                break;
        }
    }

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
