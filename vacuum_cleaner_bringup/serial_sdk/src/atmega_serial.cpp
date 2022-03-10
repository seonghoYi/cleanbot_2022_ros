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
    MOTOR_STOP = 0x00,
    MOTOR_RUN,
    MOTOR_SET_LEFT_SPEED,
    MOTOR_SET_RIGHT_SPEED,
    MOTOR_SET_LEFT_DIR,
    MOTOR_SET_RIGHT_DIR,
    BT_SET_CONFIG_MODE,
    BT_CLEAR_CONFIG_MODE,
    BT_WRITE,
    SUCTION_MOTOR_RUN,
    SUCTION_MOTOR_STOP,
    SERVO_WRITE
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

enum
{
	ROS_PKT_SYNC1 = 0,
	ROS_PKT_SYNC2,
	ROS_PKT_LEN1,
	ROS_PKT_LEN2,
	ROS_PKT_LENCHECK,
	ROS_PKT_ID1,
	ROS_PKT_ID2
};

enum
{
	ROS_STATE_SYNC1 = 0,
	ROS_STATE_SYNC2,
	ROS_STATE_LEN1,
	ROS_STATE_LEN2,
	ROS_STATE_LENCHECK,
	ROS_STATE_ID1,
	ROS_STATE_ID2,
	ROS_STATE_MSGS,
	ROS_STATE_CS
};



serial::Serial *p_driver;
std::string dev_;
std::uint32_t baudrate_;
std::uint32_t buf_len_;

std::uint8_t *p_packet_buf_;


bool send_inst(std::uint8_t id, std::uint8_t inst, std::uint8_t *param, uint16_t len);



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
    /*
    for (int i = 0; i < index; i++)
    {
        printf("%X\n", p_packet_buf_[i]);
    }
    */

    p_driver->write(p_packet_buf_, index);
    return ret;
}

bool receive_packet(ros_t *p_ros)
{
    bool ret = false;
	std::uint8_t rx_data;
	std::uint8_t index;
	std::uint32_t buf;
	/*
	if (p_ros->is_open != true)
	{
		return false;
	}
    */


	if (p_driver->available() > 0)
	{
        //printf("ok\n");
		//rx_data = p_ros->driver.available(p_ros->ch);
		//p_ros->driver.write(p_ros->ch, &rx_data, 1);
		rx_data = p_driver->read();
        //printf("%X\n", rx_data);
		//p_ros->driver.write(p_ros->ch, &rx_data, 1);
		//uartPrintf(_DEF_UART0, "%X\n", rx_data);
	}
	else
	{
		return false;
	}

    //printf("%X\n", p_ros->state);
	switch(p_ros->state)
	{
		case ROS_STATE_SYNC1:
			if (rx_data == 0xFF)
			{
				p_ros->packet_buf[ROS_PKT_SYNC1] = rx_data;
				p_ros->state = ROS_STATE_SYNC2;
			}
			else
			{
				p_ros->state = ROS_STATE_SYNC1;
			}
			break;
		case ROS_STATE_SYNC2:
			if (rx_data == 0xFF)
			{
				p_ros->packet_buf[ROS_PKT_SYNC2] = rx_data;
				p_ros->state = ROS_STATE_LEN1;
			}
			else
			{
				p_ros->state = ROS_STATE_SYNC1;
			}
			break;
		case ROS_STATE_LEN1:
			p_ros->packet_buf[ROS_PKT_LEN1] = rx_data;
			p_ros->state = ROS_STATE_LEN2;
			break;
		case ROS_STATE_LEN2:
			p_ros->packet_buf[ROS_PKT_LEN2] = rx_data;
			p_ros->state = ROS_STATE_LENCHECK;
			break;
		case ROS_STATE_LENCHECK:
			p_ros->packet_buf[ROS_STATE_LENCHECK] = rx_data;
			
			buf = p_ros->packet_buf[ROS_PKT_LEN2] + p_ros->packet_buf[ROS_PKT_LEN1];
			p_ros->packet.msg_len_checksum = (std::uint8_t)255 - (std::uint8_t)(buf % 256);
			//printf("%X\n", p_ros->packet.msg_len_checksum);
			//printf("%X\n", buf);
            if (p_ros->packet.msg_len_checksum == p_ros->packet_buf[ROS_PKT_LENCHECK])
			{
				p_ros->state = ROS_STATE_ID1;
			}
			else
			{
				p_ros->state = ROS_STATE_SYNC1;
			}
			
			p_ros->packet.msg_len =	(p_ros->packet_buf[ROS_PKT_LEN1] << 0) & 0x00FF;
			
			p_ros->packet.msg_len |= (p_ros->packet_buf[ROS_PKT_LEN2] << 8) & 0xFF00;
			break;
		case ROS_STATE_ID1:
			p_ros->packet_buf[ROS_STATE_ID1] = rx_data;
			p_ros->state = ROS_STATE_ID2;
			break;
		case ROS_STATE_ID2:
			p_ros->packet_buf[ROS_STATE_ID2] = rx_data;
			p_ros->index = 7;
			p_ros->packet.msgs = &p_ros->packet_buf[7];
			if (p_ros->packet.msg_len > 0)
			{
				p_ros->state = ROS_STATE_MSGS;
			}
			else
			{
				p_ros->state = ROS_STATE_SYNC1;
				ret = true;
			}
			break;
		case ROS_STATE_MSGS:
			index = p_ros->index;
			p_ros->index++;
			p_ros->packet_buf[index] = rx_data;
			if (p_ros->index >= p_ros->packet.msg_len + 7)
			{
				p_ros->state = ROS_STATE_CS;
				/*
				p_ros->state = ROS_STATE_SYNC1;
				p_ros->index = 0;
				ret = true;
				*/
				//p_ros->driver.write(p_ros->ch, p_ros->packet_buf, p_ros->packet.msg_len + 7);
			}
			break;
		case ROS_STATE_CS:
			index = p_ros->index;
			p_ros->packet_buf[index] = rx_data;
			buf = 0;
			for (int i = 0; i < p_ros->packet.msg_len; i++)
			{
				buf += p_ros->packet_buf[7 + i];
			}
			p_ros->packet.checksum = 255 - (uint8_t)(buf % 256);
			if (p_ros->packet_buf[index] == p_ros->packet.checksum)
			{
				ret = true;
				p_ros->state = ROS_STATE_SYNC1;
			}
			else
			{
				ret = false;
				p_ros->state = ROS_STATE_SYNC1;
			}
			
			p_ros->index = 0;
		break;
		default:
			break;
	}
	if (ret == true)
	{
		p_ros->packet.id = p_ros->packet_buf[ROS_PKT_ID2];
		p_ros->packet.inst = p_ros->packet_buf[ROS_PKT_ID1];
	}
	
	return ret;
}


bool stopMotor()
{
    return setLeftMotorSpeed(0) && setRightMotorSpeed(0) && send_inst(ID, MOTOR_STOP, NULL, 0);
}

bool runMotor()
{
    return send_inst(ID, MOTOR_RUN, NULL, 0);
}

bool setLeftMotorSpeed(std::uint8_t speed)
{
    std::uint8_t param = speed;
    return send_inst(ID, MOTOR_SET_LEFT_SPEED, &param, 1);
}

bool setRightMotorSpeed(std::uint8_t speed)
{
    std::uint8_t param = speed;
    return send_inst(ID, MOTOR_SET_RIGHT_SPEED, &param, 1);
}

bool setLeftMotorDirection(bool dir)
{
    std::uint8_t param = dir;
    return send_inst(ID, MOTOR_SET_LEFT_DIR, &param, 1);
    
}

bool setRightMotorDirection(bool dir)
{
    std::uint8_t param = dir;
    return send_inst(ID, MOTOR_SET_RIGHT_DIR, &param, 1);
}

bool setModeBTConfig()
{
    return send_inst(ID, BT_SET_CONFIG_MODE, NULL, 0);
}

bool clearModeBTConfig()
{
    return send_inst(ID, BT_CLEAR_CONFIG_MODE, NULL, 0);
}

bool writeBT(std::uint8_t *data, int len)
{
    return send_inst(ID, BT_WRITE, data, len);
}

bool runSuctionMotor()
{
    return send_inst(ID, SUCTION_MOTOR_RUN, NULL, 0);
}

bool stopSuctionMotor()
{
    return send_inst(ID, SUCTION_MOTOR_STOP, NULL, 0);
}

bool closeClamper()
{
    uint8_t l_data[] = {0, 150};
    uint8_t r_data[] = {1, 30};
    return send_inst(ID, SERVO_WRITE, l_data, 2) && send_inst(ID, SERVO_WRITE, r_data, 2);
}

bool openClamper()
{
    uint8_t l_data[] = {0, 30};
    uint8_t r_data[] = {1, 150};
    return send_inst(ID, SERVO_WRITE, l_data, 2) && send_inst(ID, SERVO_WRITE, r_data, 2);
}
