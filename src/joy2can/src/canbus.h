#ifndef CANBUS_H
#define CANBUS_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>


class CANbus
{
    public:
        // Constructor
        CANbus();

        // Destructor
        ~CANbus();

        void send_data(float ps4Data[]);

    private:
        int _socket;
        struct sockaddr_can addr;
        struct ifreq ifr;

        struct can_frame tx_cartCoord;
        struct can_frame rx_cartCoord;
};

#endif