
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdlib>
#include <thread>
#include <chrono>

#include "serialport/CRC_Check.h"
#include "serialport/SerialPort.h"
#include "global/Config.hpp"
SerialPort::SerialPort() {
    this->init();
}

SerialPort::~SerialPort() {
    this->closePort();
}

bool SerialPort::init() {
    this->fd_ = open(((std::string)J_SENSORS.config_["serialport"]["port"]).c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    J_SENSORS.config_["serialport"]["baudrate"] >> this->baudrate_;
    J_SENSORS.config_["serialport"]["databits"] >> this->databits_;
    J_SENSORS.config_["serialport"]["stopbits"] >> this->stopbits_;
    this->parity_ =  'N';

    if (this->fd_ == -1) {
        return false;
    }

    this->setBrate();

    if (this->setBit() == false) {
        exit(0);
    }

    printf("SerialPort inits successed.\n");

    return true;
}

bool SerialPort::receive(CarData &cd) {
    if (this->fd_ < 0) {
        // thread-safe throttle: print at most once per 5 seconds
        static std::mutex last_print_mutex;
        static auto last_print = std::chrono::steady_clock::now() - std::chrono::seconds(10);
        {
            std::lock_guard<std::mutex> lk(last_print_mutex);
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print).count() >= 5) {
                printf("SerialPort receive: invalid fd_=%d\n", this->fd_);
                last_print = now;
            }
        }
    // shorter sleep to avoid blocking the camera capture loop when
    // serial port is unavailable. Keep a small sleep to avoid busy-looping.
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return false;
    }
    int bytes;
    int result = ioctl(this->fd_, FIONREAD, &bytes);
    if (result == -1) {
        printf("ioctl: %s\n", strerror(errno));
        return false;
    }
    if (bytes == 0) {
        return false;
    }
    bytes = read(this->fd_, this->r_data_, bytes);
//    std::cout << "Receive Bytes: " << bytes << std::endl;
    // 每6字节一帧：前4字节数据，后2字节CRC16
    for (int i = 0; i <= bytes - 7; i++) {
        if ((unsigned char)this->r_data_[i] == 0xA5) { // 帧头A5
            uint16_t crc_recv = (uint16_t)this->r_data_[i+5] | ((uint16_t)this->r_data_[i+6] << 8); // 第6、7字节为CRC16（低高位）
            uint16_t crc_calc = Get_CRC16_Check_Sum(&this->r_data_[i+1], 4, 0xFFFF); // 计算4字节数据区CRC16
            if (crc_recv == crc_calc) {
                // copy into temporary byte array then assign to struct fields
                uint8_t tmp[4];
                memcpy(tmp, &this->r_data_[i+1], 4);
                cd.header = tmp[0];
                cd.mode = tmp[1];
                cd.status = tmp[2];
                cd.number = tmp[3];
                return true;
            }
        }
    }
    // Throttle this message to at most once every 5 seconds to avoid log flooding
    static std::mutex _nf_mutex;
    static auto _nf_last = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    {
        std::lock_guard<std::mutex> lk(_nf_mutex);
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - _nf_last).count() >= 5) {
            std::cout << "cannot find valid frame," << std::endl;
            _nf_last = now;
        }
    }
    return false;
}

void SerialPort::send(VisionData &vd) {
    if (this->fd_ < 0) {
        printf("SerialPort send: invalid fd_=%d\n", this->fd_);
        return;
    }
    memcpy(this->t_data_, &vd, sizeof(VisionData));
    Append_CRC8_Check_Sum(this->t_data_, 3);
    Append_CRC16_Check_Sum(this->t_data_, sizeof(VisionData));
    int result = write(this->fd_, this->t_data_, sizeof(VisionData));
    if (result == -1) {
//        std::cout << "send failed." << std::endl;
        this->closePort();
        this->init();
    }

}

void SerialPort::closePort() {
    if (this->fd_ >= 0) {
        close(this->fd_);
        this->fd_ = -1;
    }
}

void SerialPort::setBrate() {
    int speed_arr[] = {B921600, B460800, B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
                       B921600, B460800, B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
    };
    int name_arr[] = {921600, 460800, 115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,
                      921600, 460800, 115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,
    };

    unsigned int i;
    int status;
    struct termios Opt;     //用于存储获得的终端参数信息
    tcgetattr(this->fd_, &Opt);
    for (i = 0;  i < sizeof(speed_arr) / sizeof(int);  i++) {
        if (this->baudrate_ == name_arr[i]) {
            tcflush(this->fd_, TCIOFLUSH);                 //清空缓冲区的内容
            cfsetispeed(&Opt, speed_arr[i]);        //设置接受和发送的波特率
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(this->fd_, TCSANOW, &Opt);  //使设置立即生效

            if (status != 0) {
                return;
            }

            tcflush(this->fd_, TCIOFLUSH);

        }
    }
}

int SerialPort::setBit() {
    // 获取串口相关参数
    struct termios termios_p;
    tcgetattr(this->fd_, &termios_p);
    // CLOACL：保证程序不会占用串口
    // CREAD：使得能够从串口中读取输入数据
    termios_p.c_cflag |= (CLOCAL | CREAD);

    // 设置数据位
    termios_p.c_cflag &= ~CSIZE;
    switch (this->databits_) {
        case 7:
            termios_p.c_cflag |= CS7;
            break;

        case 8:
            termios_p.c_cflag |= CS8;
            break;

        default:
            fprintf(stderr, "Unsupported data size\n");
            return false;
    }

    // 设置奇偶校验位
    switch (this->parity_) {
        case 'n':
        case 'N':
            termios_p.c_cflag &= ~PARENB;       //PARENB:启用奇偶校验码的生成和检测功能。
            termios_p.c_iflag &= ~INPCK;        //INPCK：对接收到的字符执行奇偶校检。
            break;

        case 'o':
        case 'O':
            termios_p.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
            termios_p.c_iflag |= INPCK;             /* Disnable parity checking */
            break;

        case 'e':
        case 'E':
            termios_p.c_cflag |= PARENB;        /* Enable parity */
            termios_p.c_cflag &= ~PARODD;       /* 转换为偶效验*/
            termios_p.c_iflag |= INPCK;         /* Disnable parity checking */
            break;

        case 'S':
        case 's':  /*as no parity*/
            termios_p.c_cflag &= ~PARENB;
            termios_p.c_cflag &= ~CSTOPB;
            break;

        default:
            fprintf(stderr, "Unsupported parity\n");
            return false;

    }

    // 设置停止位
    switch (this->stopbits_) {
        case 1:
            termios_p.c_cflag &= ~CSTOPB;
            break;

        case 2:
            termios_p.c_cflag |= CSTOPB;
            break;

        default:
            fprintf(stderr, "Unsupported stop bits\n");
            return false;

    }

    if (this->parity_ != 'n') {
        termios_p.c_iflag |= INPCK;
    }

    tcflush(this->fd_, TCIFLUSH);                                  //清除输入缓存区
    termios_p.c_cc[VTIME] = 1;                              //设置超时
    termios_p.c_cc[VMIN] = 0;                 //最小接收字符
    termios_p.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);   //Input原始输入
    // 写死了，不管前面写了什么，都一定是无奇偶校验
    termios_p.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    termios_p.c_iflag &= ~(ICRNL | IGNCR);
    termios_p.c_oflag &= ~OPOST;                            //Output禁用输出处理

    if (tcsetattr(this->fd_, TCSANOW, &termios_p) != 0) {
        return false;
    }

    return true;
}