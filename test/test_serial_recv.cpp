#include <iostream>
#include <vector>
#include <iomanip>
#include <cstdint>
#include <cstddef>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

// CRC16-IBM (Modbus)
uint16_t crc16_ibm(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

int main() {
    const char* serial_port = "/dev/ttyUSB0"; 
    int fd = open(serial_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "无法打开串口: " << serial_port << std::endl;
        return 1;
    }
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "获取串口属性失败" << std::endl;
        close(fd);
        return 1;
    }
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "设置串口属性失败" << std::endl;
        close(fd);
        return 1;
    }

    std::vector<uint8_t> buffer;
    buffer.reserve(1024);
    uint8_t temp[128];
    std::cout << "开始实时接收串口数据..." << std::endl;
    while (true) {
        ssize_t n = read(fd, temp, sizeof(temp));
        if (n > 0) {
            buffer.insert(buffer.end(), temp, temp + n);
            // 滑窗查找帧
            while (buffer.size() >= 7) {
                bool found = false;
                for (size_t i = 0; i + 7 <= buffer.size(); ++i) {
                    if (buffer[i] == 0xA5) {
                        uint16_t crc_recv = buffer[i+5] | (buffer[i+6] << 8);
                        uint16_t crc_calc = crc16_ibm(&buffer[i], 5);
                        std::cout << "[Frame] Offset " << i << ": ";
                        for (int j = 0; j < 7; ++j) std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)buffer[i+j] << " ";
                        std::cout << std::dec << " | Data: ";
                        for (int j = 0; j < 4; ++j) std::cout << (int)buffer[i+1+j] << " ";
                        std::cout << "| CRC_recv: 0x" << std::hex << crc_recv << ", CRC_calc: 0x" << crc_calc;
                        if (crc_recv == crc_calc) std::cout << " [OK]";
                        else std::cout << " [CRC ERROR]";
                        std::cout << std::endl;
                        buffer.erase(buffer.begin(), buffer.begin() + i + 7);
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    // 没有帧头，丢弃第一个字节
                    buffer.erase(buffer.begin());
                }
            }
        } else {
            usleep(10000); // 10ms
        }
    }
    close(fd);
    return 0;
}
