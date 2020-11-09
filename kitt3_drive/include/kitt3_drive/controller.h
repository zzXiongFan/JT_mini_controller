#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "string.h"
#include <fcntl.h>
#include <iostream>
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
#include <boost/date_time/posix_time/posix_time.hpp>
#include <command.h>

using namespace std;

class Controller
{
private:
    /**结构体：twist2motor命令**/
    typedef struct{
        int motor_left;
        int motor_right;
    }motor;
    /**结构体：里程计数据**/
    typedef struct{
        /**m**/
        double x;
        double y;
        /**rad -3.14~+3.14**/
        double theta;
        /**四元数**/
        double qua_x;
        double qua_y;
        double qua_z;
        double qua_w;
        float dx;
        float dz;
    }pose;
    /**结构体：编码器的值**/
    typedef struct{
        long left_encoder;
        long right_encoder;
    }encoder;
    /**两轮差速底盘的两个主动轮之间的距离**/
    float base_width;
    /**行走一米编码器的数值**/
    float ticks_meter;
    /**50ms的控制周期**/
    int duration;

    pthread_t pid;

    static void * start_thread(void* arg);// //静态成员函数

public:

    motor mot;
    encoder encoder_;
    pose pose_;
    bool publish_odom;

    long enc_left;
    long enc_right;

    int t_next;
    long now;
    /**下一个时刻时间戳**/
    long then;
    /**循环内时间间隔**/
    int elapsed;

public:
    Controller() {
        base_width = BASE_WIDTH;
        ticks_meter = TICKS_METER;
        duration = 20;
        enc_left = NULL;
        enc_right = NULL;
        mot.motor_left = 0;
        mot.motor_left = 0;
        encoder_.left_encoder = 0;
        encoder_.right_encoder = 0;
        pose_.x = 0;
        pose_.y = 0;
        pose_.theta = 0;
        pose_.dx = 0;
        pose_.dz = 0;
        pose_.qua_w = 0;
        pose_.qua_x = 0;
        pose_.qua_y = 0;
        pose_.qua_z = 0;

        publish_odom = true;
        int ret = start();
        if(ret == 0)
        {
            cout<<"Create pthread successfully!"<<endl;
        }

        now = getTime();
        then = now;
        t_next = now + duration;
    }
    int start();
    /**讲twist类型的消息转化为左右电机的控制命令**/
    void twist2motor(float linear_x ,float angular_z);
    /**根据编码器生成里程计数据**/
    void produce_odom();
    /**获取系统时间**/
    const long getTime();
    /**释放资源**/
    void close_thread();
};

#endif

