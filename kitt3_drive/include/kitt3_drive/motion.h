#ifndef MOTION_H
#define MOTION_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/signal.h>
#include <netinet/in.h>
#include "string.h"
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <errno.h>
#include <vector>
#include <sstream>
#include <string>
#include <boost/lexical_cast.hpp>
#include <pthread.h>
#include <boost/bind.hpp>
#include <pthread.h>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <command.h>

using namespace std;

class InfraredSerial {
    int serial_fd;
    typedef struct
    {
        /**encoders**/
        double left_encoder;
        double right_encoder;
        /**转速**/
        int left_speed; // r/min
        int right_speed;
        /**左右电机母线电流**/
        int left_electric; // ma
        int right_electric;
        /**电池电压**/
        int voltage;
    } robot_info;

    double first_left_encoder,first_right_encoder;
    /**serial is opened**/
    bool isOpenned;

    vector<char> serial_buff;

public:
    robot_info information;

    pthread_t pid;
    static void * start_thread(void* arg);// //静态成员函数
    int start();

    InfraredSerial():serial_fd(0){
        /**初始化**/
        information.left_electric = 0;
        information.left_encoder = 0;
        information.left_speed = 0;
        information.right_electric = 0;
        information.right_encoder = 0;
        information.right_speed = 0;
        information.voltage = 0;

        first_left_encoder = 0;
        first_right_encoder = 0;

        isOpenned = false;

        serial_fd = open(DEV, O_RDWR | O_NOCTTY | O_NDELAY);

        if (serial_fd < 0) {
            perror("open");
            isOpenned = false;
        }else {
            isOpenned = true;
        }
        struct termios options;

        tcgetattr(serial_fd, &options);
        /**2. 修改所获得的参数*/
        options.c_cflag |= (CLOCAL | CREAD);//设置控制模式状态，本地连接，接收使能
        options.c_iflag &= ~(ICRNL | IXON);
        options.c_cflag &= ~CSIZE;//字符长度，设置数据位之前一定要屏掉这个位
        options.c_cflag &= ~CRTSCTS;//无硬件流控
        options.c_cflag |= CS8;//8位数据长度
        options.c_cflag &= ~CSTOPB;//1位停止位
        options.c_iflag |= IGNPAR;//无奇偶检验位
        options.c_oflag = 0; //输出模式
        options.c_lflag = 0; //不激活终端模式
        cfsetospeed(&options, B115200);//设置波特率
        cfsetispeed(&options, B115200);
        /**3. 设置新属性，TCSANOW：所有改变立即生效*/
        tcflush(serial_fd, TCIFLUSH);//溢出数据可以接收，但不读
        tcsetattr(serial_fd, TCSANOW, &options);

        int ret = start();
        if(ret == 0)
        {
            cout<<"Create Motion pthread successfully!"<<endl;
        }
    }

//    ~InfraredSerial();

    int uart_send(char *data, ssize_t datalen);

    int uart_recv(unsigned char *data, int datalen);

    void uart_write(int left , int right);

    void uart_explain();

    void printSerial(unsigned char* data,int len);

    void closeUart();

protected:

};

#endif

