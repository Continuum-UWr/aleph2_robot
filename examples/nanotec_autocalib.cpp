#include <chrono>
#include <cstdint>
#include <thread>
#include <unistd.h>

#include "nanotec_driver/nanotec.h"

int Auto(Nanotec *nanotec);
int Target(Nanotec *nanotec, int32_t target);
int Quit(Nanotec *nanotec);
void Save(Nanotec *nanotec);
void Reset(Nanotec *nanotec);

int main(int argc, char **argv)
{
    uint8_t node_id = 2;
    std::string busname = "can0";
    std::string baudrate = "500K";  
    int32_t target = -100;
    
    char option = 's';
    bool Exit = false;
    std::string line = "abc";
        
    std::cout << "Put node's id. (" << node_id << ")" << std::endl;
    getline(std::cin, line);
    if(line != "")
    {
        node_id = static_cast<uint8_t>(std::stoi(line));
    }
    
    std::cout << "Put busname. (" << busname << ")" << std::endl;
    getline(std::cin, line);
    if(line != "")
    {
        busname = line;
    }

    std::cout << "Put baudrate. (" << baudrate << ")"<< std::endl;
    std::cin.ignore();
    getline(std::cin, line);
    if(line != "")
    {
        baudrate == line;
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

    int mode = 0;
    std::cout << "Select operation mode. (VELOCITY)" << std::endl;
    std::cout << "VELOCITY. (1)" << std::endl;
    std::cout << "POSITION. (2)" << std::endl;
    std::cout << "TORQUE. (3)" << std::endl;
    std::cin >> mode;

    Nanotec *nanotec;

    if(mode == 3)
    {
       nanotec = new Nanotec(device, Nanotec::OperationMode::TORQUE); 
    }
    else if(mode == 2)
    {
       nanotec = new Nanotec(device, Nanotec::OperationMode::POSITION);
    }
    else
    {
        nanotec = new Nanotec(device, Nanotec::OperationMode::VELOCITY);
    }

    while(!Exit)
    {    
        std::cout << "Choose your option:" << std::endl;
        std::cout << "Autocalib. (a)" << std::endl;
        std::cout << "Set target. (t)" << std::endl;
        std::cout << "Save parameters. (s)" << std::endl;
        std::cout << "Reset parameters. (r)" << std::endl; 
        std::cout << "Quit. (q)" << std::endl;

        std::cin >> option;
        if(option == 'a')
        {
            Auto(nanotec);
        }
        else if(option == 't')
        {
            std::cout << "Remember that nominal_current, peak_current and peak_length in SetMotorProtection function are already set." << std::endl;
            std::cout << "target: " << target << std::endl;
            std::cout << "Put target." << std::endl;
            std::cin.ignore();
            getline(std::cin, line);
            if(line != "")
            {
                target = static_cast<int32_t>(std::stoi(line));
            }
            Target(nanotec, target);
        }
        else if(option == 's')
        {
            Save(nanotec);
        }
        else if(option == 'r')
        {
            Reset(nanotec);
        }
        else if(option == 'q')
        {
            Quit(nanotec);
            Exit = !Exit;
        }
    }
}

int Auto(Nanotec *nanotec)
{
    std::map<std::string, int64_t> params = nanotec->Autocalib();

    std::cout << "Autocalib completed!" << std::endl;
    std::cout << "Parameters:" << std::endl;

    for (auto const& x : params)
    {
        std::cout << "\"" << x.first << "\": " << x.second << std::endl;  
    }
}

int Target(Nanotec *nanotec, int32_t target)
{
    nanotec->SetPowerMode(Nanotec::PowerMode::ACTIVE);
    nanotec->SetMotorProtection(2000, 2500, 100);
    nanotec->SetTarget(target);
}

int Quit(Nanotec *nanotec)
{
    nanotec->SetPowerMode(Nanotec::PowerMode::OFF);
}

void Save(Nanotec *nanotec)
{
    std::cout << "To be done." << std::endl;
}

void Reset(Nanotec *nanotec)
{
    std::cout << "To be done." << std::endl;
}

