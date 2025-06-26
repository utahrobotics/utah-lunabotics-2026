/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "example.h"
#include "iox/duration.hpp"
#include "iox2/log.hpp"
#include "iox2/node.hpp"
#include "iox2/sample_mut.hpp"
#include "iox2/service_name.hpp"
#include "iox2/service_type.hpp"
#include "transmission_data.hpp"

int main(int argc, char *argv[])
{

    // Initialize
    UnitreeLidarReader *lreader = createUnitreeLidarReader();

    std::string port = "/dev/ttyACM0";
    uint32_t baudrate = 4000000;

    if (lreader->initializeSerial(port, baudrate))
    {
        printf("Unilidar initialization failed! Exit here!\n");
        exit(-1);
    }
    else
    {
        printf("Unilidar initialization succeed!\n");
    }

    lreader->startLidarRotation();
    sleep(1);
    
    // Set lidar work mode
    uint32_t workMode = 8;
    std::cout << "set Lidar work mode to: " << workMode << std::endl;
    lreader->setLidarWorkMode(workMode);
    sleep(1);

    // Reset Lidar
    lreader->resetLidar();
    sleep(1);

    // Process
    exampleProcess(lreader);
    
    return 0;
}