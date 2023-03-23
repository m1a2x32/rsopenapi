#include <iostream>
#include <unistd.h>
#include "RtDB2.h"
#include "comm.hpp"
#include "Service.hpp"
#include "boost/program_options.hpp"
#include <Controller.hpp>

namespace po = boost::program_options;

uint64_t string2uint(const std::string& s)
{
    if (s.find("0x") == 0)
    {
        uint64_t result;
        std::stringstream ss;
        ss << std::hex << s;
        ss >> result;
        return result;
    }
    return std::stoull(s);
}

int main(int argc, char **argv)
{
    int robotid;
    std::string hashstr;

    // Create program   options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("hash,h", po::value<std::string>(&hashstr)->default_value(""), "hash")
        ("robot,r", po::value<int>(&robotid)->default_value(0), "robot");

    // Parse options
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // Extract hash
    uint64_t hash = string2uint(hashstr);
    std::cout << "Target robot: " << robotid << std::endl;
    std::cout << "Using robot hash: " << std::hex << hash << std::dec << std::endl;

    // Start communication service
    rsopen::Service service;
    service.start();

    // Create proxy to robot
    RobotController rob(robotid, hash);

    float x = 0, y = 0;
    bool goalReached = true;
    while (true)
    {
        if(goalReached)
        {
            std::cout << "Enter x dest: " << std::endl;
            std::cin >> x;
            std::cout << "Enter y dest: " << std::endl;
            std::cin >> y;
            goalReached = false;
        }
        else
        {
            goalReached = rob.moveToPoint(pos2d(x,y));
        }
    }

    return 0;
}
