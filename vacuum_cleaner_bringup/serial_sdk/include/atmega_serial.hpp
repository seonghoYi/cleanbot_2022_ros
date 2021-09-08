#include <string>
#include <cstdint>

#include "serial.hpp"



/*
namespace hw_control
{
class Controller
{
public:
    Controller(std::string &dev, std::uint32_t baudrate, std::uint32_t buf_len);
    ~Controller();

    bool stop();
    bool hold();
    bool advance(std::uint8_t speed);
    bool reverse(std::uint8_t speed);
    bool turn(std::string direction, std::uint8_t speed);
    bool steering(std::uint8_t steering_angle, std::uint8_t speed);
    bool left_motor_config(std::uint8_t speed, bool dir);
    bool right_motor_config(std::uint8_t speed, bool dir);
    bool motor_config(std::uint8_t L_speed, bool L_dir, std::uint8_t R_speed, bool R_dir);

private:
    serial::Serial *p_driver;
    std::string dev_;
    std::uint32_t baudrate_;
    std::uint32_t buf_len_;
    
    std::uint8_t *p_packet_buf_;
    

    bool send_inst(std::uint8_t id, std::uint8_t inst, std::uint8_t *param, uint16_t len);
    bool receive_packet();
};
}
*/

void initMotor(std::string &port, int baudrate, int buf_len);
void deinitMotor();
bool stopMotor();
bool holdMotor();
bool advanceMotor(std::uint8_t speed);
bool reverseMotor(std::uint8_t speed);
bool turnMotor(std::string direction, std::uint8_t speed);
bool steeringMotor(std::uint8_t steering_angle, std::uint8_t speed);
bool configLeftMotor(std::uint8_t speed, bool dir);
bool configRightMotor(std::uint8_t speed, bool dir);
bool configMotor(std::uint8_t L_speed, bool L_dir, std::uint8_t R_speed, bool R_dir);
