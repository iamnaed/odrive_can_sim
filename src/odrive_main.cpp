#include <string>
#include <iostream>

#include "odrive_can_sim.hpp"

using namespace odrive_can_sim;

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    std::cout << "Odrive simulation started\r\n";

    // Initialize
    std::string can_name = "vcan0";
    std::array<int, 6> joint_can_ids = {1, 2, 3, 4, 5, 6};
    int encoder_broadcast_ms = 10;

    // Create Odrive object
    OdriveCanSim odrv_(can_name, joint_can_ids, encoder_broadcast_ms);
    if (!odrv_.connect())
    {
        printf("Odrive failed to connect to CAN bus\r\n");
        return 0;
    }
    odrv_.spin();

    std::cout << "Odrive simulation ended\r\n";
    return 0;
}