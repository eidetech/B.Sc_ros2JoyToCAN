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

#ifndef PI
#define PI 3.14159265359
#endif


class CANbus
{
    public:
        // Constructor
        CANbus();

        // Destructor
        ~CANbus();

        void send_data(float ps4Data[]);
        void send_spray_status(float sprayStatus);
        float read_IMU_pitch();

        float pitch;

    private:
        int _socket;
        struct sockaddr_can addr;
        struct ifreq ifr;

        struct can_frame tx_cartCoord;
        struct can_frame rx_cartCoord;

        struct can_frame tx_sprayStatus;

        struct can_frame rx_IMU;
};

#endif