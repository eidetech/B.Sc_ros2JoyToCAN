#include "canbus.h"
#include "rclcpp/rclcpp.hpp"
#include <bitset>

CANbus::CANbus()
{
    if ((this->_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Socket error");
    }

    strcpy(ifr.ifr_name, "can0");
    ioctl(this->_socket, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(this->_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind error");
    }

    
    this->tx_cartCoord.can_id = 0x01; // ID of CAN message
    this->tx_cartCoord.can_dlc = 8; // Size of payload

    this->rx_cartCoord.can_id = 0x02; // ID of CAN message
    this->rx_cartCoord.can_dlc = 8; // Size of payload

    this->tx_sprayStatus.can_id = 0x409; // ID of CAN message for sending spray status
    this->tx_sprayStatus.can_dlc = 8; // Size of payload

    this->tx_pitch_sp.can_id = 0x200; // ID of CAN message for sending pitch setpoint
    this->tx_pitch_sp.can_dlc = 8; // Size of payload

}


CANbus::~CANbus()
{
    if (close(this->_socket) < 0)
    {
        perror("Close CAN socket");
    }
}


void CANbus::send_data(float ps4Data[])
{
    uint32_t ps4OutData[8];
    uint8_t sign_frame1 = 0b00000000; // 0 = +, 1 = -

    for(int i = 0; i < 8; i++)
    {
        if(ps4Data[i] < 0)
        {
            ps4OutData[i] = (uint32_t)(ps4Data[i]*-1000000.);
            sign_frame1 |= (1U << i);
            //std::cout << ps4OutData[i] << std::endl;
        }else
        {
            ps4OutData[i] = (uint32_t)(ps4Data[i]*1000000.);
            //std::cout << ps4OutData[i] << std::endl;
        }
        
    }

    this->tx_cartCoord.data[0] = ps4OutData[0];
    this->tx_cartCoord.data[1] = ps4OutData[0] >> 8;
    this->tx_cartCoord.data[2] = ps4OutData[0] >> 16;
    this->tx_cartCoord.data[3] = ps4OutData[1];
    this->tx_cartCoord.data[4] = ps4OutData[1] >> 8;
    this->tx_cartCoord.data[5] = ps4OutData[1] >> 16;
    this->tx_cartCoord.data[6] = sign_frame1;
    //this->tx_cartCoord.data[7] = ps4OutData[7]; // unused

    
    uint32_t joyX = tx_cartCoord.data[0] | tx_cartCoord.data[1]<<8 | tx_cartCoord.data[2]<<16;
    uint32_t joyY = tx_cartCoord.data[3] | tx_cartCoord.data[4]<<8 | tx_cartCoord.data[5]<<16;

    std::cout << "Received joyX:" << joyX << std::endl;
    std::cout << "Received joyY:" << joyY << std::endl;
    std::cout << "Received sign:";

    std::bitset<8> y(sign_frame1);
    std::cout << y << '\n';

    write(this->_socket, &this->tx_cartCoord, sizeof(struct can_frame));
}

void CANbus::send_spray_status(float sprayStatus)
{
    this->tx_sprayStatus.data[0] = (int)sprayStatus;
    for (int i = 1; i < 8; i++)
    {
        this->tx_sprayStatus.data[i] = 0;
    }
    write(this->_socket, &this->tx_sprayStatus, sizeof(struct can_frame));
}

void CANbus::read_IMU_data()
{
    read(this->_socket,&this->rx_IMU, sizeof(struct can_frame));
    if(rx_IMU.can_id == 0x80)
    {
        uint16_t rx_pitch = rx_IMU.data[0] | rx_IMU.data[1] << 8;
        pitch = (float)rx_pitch;
        pitch = ((pitch-10000)/10000)*180/PI;

        uint16_t rx_roll = rx_IMU.data[2] | rx_IMU.data[3] << 8;
        roll = (float)rx_roll;
        roll = ((roll-10000)/10000)*180/PI;

        uint16_t rx_yaw = rx_IMU.data[4] | rx_IMU.data[5] << 8;
        yaw = (float)rx_yaw;
        yaw = ((yaw-10000)/10000)*180/PI;

        uint16_t rx_pitch_sp = rx_IMU.data[6] | rx_IMU.data[7] << 8;
        pitch_sp_readback = (float)rx_pitch_sp;
        pitch_sp_readback = ((pitch_sp_readback-20000)/1000);

        t += 0.001;

       // std::cout << "Pitch: " << pitch << ", Roll: " << roll << ", Yaw: " << yaw << ", Pitch setpoint readback: " << pitch_sp_readback << ", Time:" << t <<  std::endl;
    }
}

void CANbus::send_pitch_sp(float pitch_sp)
{
    std::cout << "pitch_sp: " << pitch_sp << std::endl;
    float pitch_sp_convert = pitch_sp * 1000.0;
    uint16_t pitch_sp_can = (int)pitch_sp_convert + 20000;
    this->tx_pitch_sp.data[0] = pitch_sp_can;
    this->tx_pitch_sp.data[1] = pitch_sp_can >> 8;

    write(this->_socket, &this->tx_pitch_sp, sizeof(struct can_frame));
}