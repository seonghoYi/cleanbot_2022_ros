#include "serial.hpp"
#include <cstdarg>
#include <wiringSerial.h>

using namespace serial;

Serial::Serial(std::string dev, std::uint32_t baudrate)
{
    dev_ = dev;
    baudrate_ = baudrate;

}

Serial::~Serial()
{
    serialClose(fd_);
}

bool Serial::open()
{
    bool ret = false;
    fd_ = serialOpen(dev_.c_str(), baudrate_);
    if (fd_ > 0)
    {
        ret = true;
    }
    return ret;
}

void Serial::close()
{
    serialClose(fd_);
}

void Serial::write(std::uint8_t *p_data, std::uint32_t length)
{
    for (int i = 0; i < length; i++)
    {
        serialPutchar(fd_, (char)p_data[i]);
    }
}

void Serial::print(char *fmt, ...)
{
        char *buf = new char[256];
        va_list args;
        int len;

        va_start(args, fmt);
        len = vsnprintf(&buf[0], 256, fmt, args);
        Serial::write((std::uint8_t *)buf, len);

        delete []buf;
}

std::uint8_t Serial::read(void)
{
    std::uint8_t ret;
    ret = serialGetchar(fd_);
    return ret;
}
int Serial::available(void)
{
    int ret;
    ret = serialDataAvail(fd_);
    return ret;
}

void Serial::flush(void)
{
    serialFlush(fd_);
}