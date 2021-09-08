#include <string>
#include <cstdint>

namespace serial
{
class Serial
{
public:
    Serial(std::string dev, std::uint32_t baudrate);
    virtual ~Serial();
    bool open(void);
    void close(void);
    void write(std::uint8_t *p_data, std::uint32_t length);
    void print(char *fmt, ...);
    std::uint8_t read(void);
    int available(void);
    void flush(void);
    
private:
    std::string dev_;
    int baudrate_;
    int fd_;
};
}
