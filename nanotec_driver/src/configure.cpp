#include <chrono>
#include <cstdint>
#include <thread>
#include <unistd.h>
#include <exception>
//#include <typeinfo>
#include <boost/program_options.hpp>

#include "nanotec_driver/nanotec.h"
#include "nanotec_driver/utils.h"

namespace po = boost::program_options;

class ConfigurationUI
{
public:
    po::variables_map mOptions;
    kaco::Master mMaster;
    kaco::Device* pDevice;

    ConfigurationUI(int argc, char **argv);
    int run();

private:
    kaco::Device& initializeDevice(kaco::Master &master, uint8_t nodeId);
};

int main(int argc, char **argv)
{
    ConfigurationUI prog(argc, argv);
    return prog.run();
}

ConfigurationUI::ConfigurationUI(int argc, char **argv)
{
    po::options_description desc;
    std::ostringstream helpMessageStream;

    helpMessageStream << "usage: rosrun nanotec_driver configure [options]" << std::endl
                      << std::endl
                      << "Before using that program you should set proper type of can conneciton by command:" << std::endl
                      << "\t sudo ip link set [busname] up type can bitrate [baudrate]"
                      << std::endl;

    desc.add_options()("help", "shows this message");
    desc.add_options()("node,n", po::value<uint>()->default_value(2), "Driver's node id on CAN line");
    desc.add_options()("busname,d", po::value<std::string>()->default_value("can0"), "Check by ip link");
    desc.add_options()("bitrate,b", po::value<std::string>()->default_value("500K"), "CAN bitrate");

    helpMessageStream << "options: " << std::endl
                      << desc << std::endl;
    std::string helpMessage = helpMessageStream.str();

    try
    {
        po::store(po::parse_command_line(argc, argv, desc), mOptions);
        po::notify(mOptions);
    }
    catch (const std::exception &e)
    {

        std::cerr << "Error: " << e.what() << std::endl
                  << std::endl;
        std::cout << helpMessage;
        throw std::string("Parsing arguments failed.");
    }

    if (mOptions.count("help"))
    {
        std::cout << helpMessage;
        throw std::string("Program should finish.");
    }

    std::string busname = mOptions["busname"].as<std::string>();
    std::string baudrate = mOptions["bitrate"].as<std::string>();

    if (!mMaster.start(busname, baudrate))
        throw std::string("Error: Starting master node on CAN was not possible.");

    mDevice = initializeDevice(mMaster, static_cast<uint8_t>(mOptions["node"].as<int>()));
}

kaco::Device& ConfigurationUI::initializeDevice(kaco::Master &master, uint8_t nodeId)
{
    bool found_device = false;
    size_t device_index;

    while (!found_device)
    {
        for (size_t i = 0; i < master.num_devices(); ++i)
        {
            kaco::Device &device = master.get_device(i);
            if (device.get_node_id() == nodeId)
            {
                found_device = true;
                device_index = i;
                break;
            }
        }
        if (found_device)
            break;

        std::cout
            << "Device with ID " << (unsigned)node_id
            << " has not been found yet. Will keep retrying. Press Ctrl+C to abort."
            << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    kaco::Device* device = &master.get_device(device_index);

    device->start();
    device->load_dictionary_from_library();
    return device;
}

int ConfigurationUI::run(){
    return 0;
}