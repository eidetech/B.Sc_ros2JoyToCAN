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

        void receive_propeller();
        void send_propeller(int idle_speed, int counterforce_speed);

        void receive_pid();
        void send_pid(float kp, float ki, float kd);

        void receive_can();

        float pitch = 0;
        float roll = 0;
        float yaw = 0;
        float pitch_sp_readback = 0;

        int rx_idle_speed = 0;
        int rx_counterforce_speed = 0;

        float rx_kp = 0;
        float rx_ki = 0;
        float rx_kd = 0;

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

        struct can_frame rx_propeller;
        struct can_frame tx_propeller;

        struct can_frame rx_pid;
        struct can_frame tx_pid;

        struct can_frame rx_can;
};

#endif