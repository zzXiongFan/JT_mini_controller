#ifndef IMU_H
#define IMU_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <serial/serial.h>
#include <serial/v8stdint.h>
#include <pthread.h>
#include "namesapce_v.h"

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

using namespace std;

class IMU
{
public:
    youi_robot_namespace::RobotImuData imuData;

    bool isOpened;

    // 实例化串口函数
    serial::Serial imu_serial;
    // imu数据读取命令
    uint8_t tx_buffer[5];
    // 体加速度清零
    uint8_t zero_euler[5];
    uint8_t zero_euler_rx[6];
    // 串口读取buff
    uint8_t rx_buffer[30];
    uint8_t rx_header[1];
    // 存储九轴数据
    int imu_buffer[54];

public:

    pthread_t pid_;

    static void * start_thread(void* arg);// //静态成员函数
    int start();

    // 构造函数
    IMU();

    // 析构函数
    ~IMU();

    /**
     * @brief 对串口数据进行解析
     * @return　无返回
     */
    void explain_line();

    /**
     * @brief 打印串口数据
     *
     * @param data     串口数据buffer
     * @param len      buffer的长度
     */
    void printSerial(uint8_t *data, int8 len);
    /**
     * @brief 将航向角清零
     *
     * @param times 尝试次数
     * @return 返回1表示已经重置，返回0表示没有成功重置
     */
    int8 resetUart(int8 times);
    /**
     * @brief 关闭串口，恢复传感器数据
     *
     * @return 无返回
     */
    void closeUart();

};
#endif
