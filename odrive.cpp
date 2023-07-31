
#include "odrive_can_sim.hpp"
#include <string>

int main(int argc, char** argv)
{
    printf("Odrive simulation started\r\n");

    std::string can_name = "vcan0";
    std::array<int, 6> joint_can_ids = {1,2,3,4,5,6};
    int encoder_broadcast_ms = 10;

    OdriveCanSim odrv_(can_name, joint_can_ids, encoder_broadcast_ms);
    if(!odrv_.connect())
    {
        printf("Odrive failed to connect to CAN bus\r\n");
    }
    odrv_.spin();
    printf("Odrive simulation ended\r\n");
    return 0;
}