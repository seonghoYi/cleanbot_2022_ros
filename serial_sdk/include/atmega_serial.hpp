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

const int ROS_PACKET_BUF_MAX    = 512;
const int ROS_MAX_SERVICE       = 128;

typedef struct
{
	std::uint8_t		sync;
	std::uint8_t		protocol_version;
	std::uint16_t	msg_len;
	std::uint8_t		msg_len_checksum;
	std::uint8_t		id;
	std::uint8_t		inst;
	std::uint8_t		*msgs;
	std::uint8_t		checksum;
} ros_packet_t;


typedef struct
{
	bool			is_open;
	std::uint8_t			ch;
	std::uint32_t		baud;
	std::uint8_t			state;
	std::uint8_t			index;
	std::uint32_t		pre_time;
	
	ros_packet_t	packet;
	std::uint8_t			packet_buf[ROS_PACKET_BUF_MAX];
	
	void (*func[ROS_MAX_SERVICE])(uint8_t *params);
	std::uint8_t			service_index;
} ros_t;



void initMotor(std::string &port, int baudrate, int buf_len);
void deinitMotor();

bool receive_packet(ros_t *p_ros);

bool stopMotor();
bool runMotor();
bool setLeftMotorSpeed(std::uint8_t speed);
bool setRightMotorSpeed(std::uint8_t speed);
bool setLeftMotorDirection(bool dir);
bool setRightMotorDirection(bool dir);
bool setModeBTConfig();
bool clearModeBTConfig();
bool writeBT(std::uint8_t *data, int len);
bool runSuctionMotor();
bool stopSuctionMotor();
bool closeClamper();
bool openClamper();