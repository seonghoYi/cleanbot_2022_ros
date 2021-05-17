#include "atmega_serial.hpp"
#include <stdio.h>

//using namespace hw_control;


enum
{
    SRL_PKT_SYNC1 = 0x00,
    SRL_PKT_SYNC2,
    SRL_PKT_LEN1,
    SRL_PKT_LEN2,
    SRL_PKT_LENCHECK,
    SRL_PKT_ID1,
    SRL_PKT_ID2
};

enum
{
    STOP = 0x00,
    HOLD,
    ADVANCE,
    REVERSE,
    TURN,
    STEERING,
    LMCONFIG,
    RMCONFIG,
    MTCONFIG
};

const std::uint8_t ID = 0x01;

/*
Controller::Controller(std::string &dev, std::uint32_t baudrate, std::uint32_t buf_len) : buf_len_(buf_len)
{
    p_driver = new serial::Serial(dev, baudrate);
    p_packet_buf_ = new std::uint8_t[buf_len_];
    if (p_driver->open() < 0)
    {
        this->~Controller();
        return;
    }
}

Controller::~Controller()
{
    p_driver->~Serial();
    delete p_driver;
    delete[] p_packet_buf_;
}

bool Controller::send_inst(std::uint8_t id, std::uint8_t inst, std::uint8_t *param, std::uint16_t len)
{
    

    bool ret = true;
    int index;
    int checksum;
    
    p_packet_buf_[SRL_PKT_SYNC1] = 0xFF;
    p_packet_buf_[SRL_PKT_SYNC2] = 0xFF;
    p_packet_buf_[SRL_PKT_LEN1] = (len >> 0) & 0xFF;
    p_packet_buf_[SRL_PKT_LEN2] = (len >> 8) & 0xFF;
    p_packet_buf_[SRL_PKT_LENCHECK] = (std::uint8_t)(255 - (p_packet_buf_[SRL_PKT_LEN1] + p_packet_buf_[SRL_PKT_LEN2]) % 256);
    p_packet_buf_[SRL_PKT_ID1] = inst;
    p_packet_buf_[SRL_PKT_ID2] = id;

    index = 7;
    checksum = 0;

    for (int i = 0; i < len; i ++)
    {
        p_packet_buf_[index++] = param[i];
        checksum += param[i];
    }
    if (len > 0)
    {
        p_packet_buf_[index++] = (std::uint8_t)(255 - (std::uint8_t)(checksum % 256));
    }

    p_driver->write(p_packet_buf_, index);
    return ret;
}

bool Controller::receive_packet()
{
    return false;
}


bool Controller::stop()
{
    return this->send_inst(ID, STOP, NULL, 0);
}

bool Controller::hold()
{
    return this->send_inst(ID, HOLD, NULL, 0);
}

bool Controller::advance(std::uint8_t speed)
{
    std::uint8_t param = speed;
    return this->send_inst(ID, ADVANCE, &param, 1);
}

bool Controller::reverse(std::uint8_t speed)
{
    std::uint8_t param = speed;
    return this->send_inst(ID, REVERSE, &param, 1);
}

bool Controller::turn(std::string direction, std::uint8_t speed)
{
    std::uint8_t param[2];
    if (direction == "CW")
    {
        param[0] = 1;
    }
    else if (direction == "CCW")
    {
        param[0] = 0;
    }
    else
    {
        return false;
    }
    param[1] = speed;

    return this->send_inst(ID, TURN, param, 2);
    
}

bool Controller::steering(std::uint8_t steering_angle, std::uint8_t speed)
{
    std::uint8_t param[2];
    if (steering_angle < -100 || steering_angle > 100)
    {
        return false;
    }spin spinonce
    param[0] = steering_angle;
    param[0] = speed;
    return this->send_inst(ID, STEERING, param, 2);
}

bool Controller::left_motor_config(std::uint8_t speed, bool dir)
{
    std::uint8_t param[2] = {speed, dir};
    return this->send_inst(ID, LMCONFIG, param, 2);
}

bool Controller::right_motor_config(std::uint8_t speed, bool dir)
{
    std::uint8_t param[2] = {speed, dir};
    return this->send_inst(ID, RMCONFIG, param, 2);
}

bool Controller::motor_config(std::uint8_t L_speed, bool L_dir, std::uint8_t R_speed, bool R_dir)
{
    std::uint8_t param[4] = {L_speed, L_dir, R_speed, R_dir};
   
    return this->send_inst(ID, MTCONFIG, param, 4);
}
*/


serial::Serial *p_driver;
std::string dev_;
std::uint32_t baudrate_;
std::uint32_t buf_len_;

std::uint8_t *p_packet_buf_;


bool send_inst(std::uint8_t id, std::uint8_t inst, std::uint8_t *param, uint16_t len);
bool receive_packet();



void initMotor(std::string &port, int baudrate, int buf_len)
{
    printf("%s, %d\n", port.c_str(), baudrate);
    p_driver = new serial::Serial(port, baudrate);
    p_packet_buf_ = new std::uint8_t[buf_len_];
    if (p_driver->open() < 0)
    {
        printf("error\n");
        deinitMotor();
        return;
    }
    printf("success\n");
}

void deinitMotor()
{
    p_driver->~Serial();
    delete p_driver;
    delete[] p_packet_buf_;
}

bool send_inst(std::uint8_t id, std::uint8_t inst, std::uint8_t *param, std::uint16_t len)
{
    

    bool ret = true;
    int index;
    int checksum;
    
    p_packet_buf_[SRL_PKT_SYNC1] = 0xFF;
    p_packet_buf_[SRL_PKT_SYNC2] = 0xFF;
    p_packet_buf_[SRL_PKT_LEN1] = (len >> 0) & 0xFF;
    p_packet_buf_[SRL_PKT_LEN2] = (len >> 8) & 0xFF;
    p_packet_buf_[SRL_PKT_LENCHECK] = (std::uint8_t)(255 - (p_packet_buf_[SRL_PKT_LEN1] + p_packet_buf_[SRL_PKT_LEN2]) % 256);
    p_packet_buf_[SRL_PKT_ID1] = inst;
    p_packet_buf_[SRL_PKT_ID2] = id;

    index = 7;
    checksum = 0;

    for (int i = 0; i < len; i ++)
    {
        p_packet_buf_[index++] = param[i];
        checksum += param[i];
    }
    if (len > 0)
    {
        p_packet_buf_[index++] = (std::uint8_t)(255 - (std::uint8_t)(checksum % 256));
    }

    p_driver->write(p_packet_buf_, index);
    return ret;
}

bool receive_packet()
{
    return false;
}


bool stopMotor()
{
    return send_inst(ID, STOP, NULL, 0);
}

bool holdMotor()
{
    return send_inst(ID, HOLD, NULL, 0);
}

bool advanceMotor(std::uint8_t speed)
{
    std::uint8_t param = speed;
    return send_inst(ID, ADVANCE, &param, 1);
}

bool reverseMotor(std::uint8_t speed)
{
    std::uint8_t param = speed;
    return send_inst(ID, REVERSE, &param, 1);
}

bool turnMotor(std::string direction, std::uint8_t speed)
{
    std::uint8_t param[2];
    if (direction == "CW")
    {
        param[0] = 1;
    }
    else if (direction == "CCW")
    {
        param[0] = 0;
    }
    else
    {
        return false;
    }
    param[1] = speed;

    return send_inst(ID, TURN, param, 2);
    
}

bool steeringMotor(std::uint8_t steering_angle, std::uint8_t speed)
{
    std::uint8_t param[2];
    if (steering_angle < -100 || steering_angle > 100)
    {
        return false;
    }
    param[0] = steering_angle;
    param[0] = speed;
    return send_inst(ID, STEERING, param, 2);
}

bool configLeftMotor(std::uint8_t speed, bool dir)
{
    std::uint8_t param[2] = {speed, dir};
    return send_inst(ID, LMCONFIG, param, 2);
}

bool configRightMotor(std::uint8_t speed, bool dir)
{
    std::uint8_t param[2] = {speed, dir};
    return send_inst(ID, RMCONFIG, param, 2);
}

bool configMotor(std::uint8_t L_speed, bool L_dir, std::uint8_t R_speed, bool R_dir)
{
    std::uint8_t param[4] = {L_speed, L_dir, R_speed, R_dir};
   
    return send_inst(ID, MTCONFIG, param, 4);
}