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
        void read_IMU_data();
        void send_pitch_sp(float pitch_sp);

        float pitch = 0;
        float roll = 0;
        float yaw = 0;
        float pitch_sp_readback = 0;

        float t = 0;

    private:
        int _socket;
        struct sockaddr_can addr;
        struct ifreq ifr;

        struct can_frame tx_cartCoord;
        struct can_frame rx_cartCoord;

        struct can_frame tx_sprayStatus;

        struct can_frame rx_IMU;
        struct can_frame tx_pitch_sp;
};

#endif