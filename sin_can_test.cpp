#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
	std::cout << "Sin Wave tester started\r\n";
 	std::cout << "Initializing sockete\r\n"; 
	int socket_write_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if(socket_write_ < 0)
	{
	    socket_write_ = 0;
	    return 1;
	}

 	std::cout << "Specifying CAN interface device\r\n"; 
	int ret_w;
        struct ifreq ifr_write;
        strcpy(ifr_write.ifr_name, "can0");
        ret_w = ioctl(socket_write_, SIOCGIFINDEX, &ifr_write);
        if (ret_w < 0) {
            socket_write_ = 0;
            return false;
        }

 	std::cout << "Binding the socket to the interface\r\n"; 
        struct sockaddr_can addr_write;
        addr_write.can_family = AF_CAN;
        addr_write.can_ifindex = ifr_write.ifr_ifindex;
        ret_w = bind(socket_write_, (struct sockaddr *)&addr_write, sizeof(addr_write));
        if (ret_w < 0) {
            socket_write_ = 0;
            return false;
        }

	setsockopt(socket_write_, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);	
	
	float amp = 48.0;
	float t = 0;
	float pi = 3.1415;
	float f = 0.1;
	float dt = 10.0;
	uint8_t data[8] {};
 	std::cout << "Sin wave generator started on can0\r\n"; 
	while(true)
	{
		float pos;

		// y = A * sin(2*PI*f*t)
		pos = amp * sin(2 * pi * f * t);
		
        // CAN send [0x009] encoder estimates
        struct can_frame frame;
        int axis_id = 0x000;
        int command_id = 0x00C;;
        std::memcpy(&data[0], &pos, 4);
        frame.can_id = (axis_id << 5) | (command_id);
        frame.len = 8;
        std::memcpy(frame.data, data, sizeof(data));

        int nbytes = write(socket_write_, &frame, sizeof(frame)); 
        if(nbytes < 0)
            continue;
		
		// Write at 100Hz [10ms]
		t+=(dt * 0.001);
		int tdt = static_cast<int>(dt);
		std::this_thread::sleep_for(std::chrono::milliseconds(tdt));
	}

	return 0;
}