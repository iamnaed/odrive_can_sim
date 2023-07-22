
#include "odrive_can_sim.hpp"

int main(int argc, char** argv)
{
    printf("Odrive simulation started\r\n");    
    OdriveCanSim odrv_;
    if(!odrv_.connect())
    {
        printf("Odrive failed to connect to CAN bus\r\n");
    }
    odrv_.spin();
    printf("Odrive simulation ended\r\n");
    return 0;
}