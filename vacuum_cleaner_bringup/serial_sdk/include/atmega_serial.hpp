#include <string>
#include <cstdint>

#include "serial.hpp"


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