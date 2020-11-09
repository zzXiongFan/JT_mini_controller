#ifndef NAMESPACE_V_H
#define NAMESPACE_V_H

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <ctime>

// 下位机STM32的串口波特率
#define DIFF_PORT 115200
// IMU的串口波特率
#define IMU_PORT 115200
// BMS的串口波特率
#define BMS_PORT 9600
// 下位机STM32的串口名称
#define DIFF_DEV "/dev/trans"
// IMU的串口名称
#define IMU_DEV "/dev/imu"
// BMS的串口名称
#define BMS_DEV "/dev/bms"
// BMS2的串口名称
#define BMS2_DEV "/dev/ttyS1"
// 两个驱动轮之间的宽度m
#define BASE_WIDTH 0.49943
// 圆周率
#define PI 3.1415926
// 轮子转动一圈的脉冲数
#define TICKS_METER 509467.f //522558.9 //509423 //529899.6 
// 摄像头图像的宽度
#define IMAGEWIDTH 640
// 摄像头图像的高度
#define IMAGEHEIGHT 480
// 摄像头图像的输出频率
#define FPS 30
// 摄像头图像是否显示（opencv）
#define ISSHOW 1
// odom话题发布的消息是否具有IMU消息
#define ODOM_USE_IMU 0
// 轮子直径 mm
#define DIAMETER 125
// 电机减速比
#define REDUCTION 20

using namespace std;

/* General types */
typedef uint8_t boolean;
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int64_t int64;
typedef uint64_t uint64;
typedef float float32;
typedef double float64;

/** 命名空間 **/

namespace youi_robot_namespace
{

/**
     * 该结构表示BMS读取的电池数据
     **/
typedef struct
{
    // 电芯1的电压(mv)
    uint16 voltage_1;
    // 电芯2的电压(mv)
    uint16 voltage_2;
    // 电芯3的电压(mv)
    uint16 voltage_3;
    // 电芯4的电压(mv)
    uint16 voltage_4;
    // 电芯5的电压(mv)
    uint16 voltage_5;
    // 电芯6的电压(mv)
    uint16 voltage_6;
    // 电芯7的电压(mv)
    uint16 voltage_7;
    // 电芯8的电压(mv)
    uint16 voltage_8;
    // 电芯9的电压(mv)
    uint16 voltage_9;
    // 电芯10的电压(mv)
    uint16 voltage_10;
    // 电芯11的电压(mv)
    uint16 voltage_11;
    // 电芯12的电压(mv)
    uint16 voltage_12;
    // 电芯13的电压(mv)
    uint16 voltage_13;
    // 电芯14的电压(mv)
    uint16 voltage_14;
    // 电芯15的电压(mv)
    uint16 voltage_15;
    // 电芯温度(摄氏度）
    int16 temperature0;
    // 电芯温度
    int16 temperature1;
    // 电芯温度
    int16 temperature2;
    // 返回电芯总电压值（mv）
    uint64 totalVoltage;
    // 返回电芯实时电流值（ma）
    int64_t electricCurrent;
    // 系统充满容量（mAh）
    uint64 dumpEnergy;
    // 电池包剩余电量
    uint64 leftEnergy;
    // 电池包当前剩余电量百分比
    uint16 energyPercentage;
    // 是否在充电,0表示没有在充电，1表示在充电
    int16_t isCharging;
    // 是否在放电，0表示没有在放电，1表示在放电
    int16_t isDischarging;

} RobotBmsData;

/**
     * 该结构表示从下位机读取的状态参数
     **/
typedef struct
{
    // 左电机编码器的绝对值
    int64 left_encoder;
    // 右电机编码器的绝对值
    int64 right_encoder;
    // 左电机的转速，单位是r/min
    int16 left_speed;
    // 右电机的转速，单位是r/min
    int16 right_speed;
    // 左电机的母线电流，单位是ma
    int16 left_electric;
    // 右电机的母线电流，单位是ma
    int16 right_electric;
    //电机是否过流 true,过流
    bool drive_sta;
    // 电压
    int16 voltage;
    // 是否已经打开净化器,0表示没有打开，1表示已经打开
    int16_t AirCleaner;

} RobotDecInfo;

/**
     * 该结构表示从下位机反馈的速度
     **/
typedef struct
{
    // x方向的线速度
    float32 linearX;
    // z方向的角速度
    float32 angularZ;

} RobotVelocityData;

/**
     * 该结构表示从twist to motor命令
     **/
typedef struct
{
    // 左电机的控制命令
    short motor_left;
    // 右电机的控制命令
    short motor_right;

} RobotMotorCmd;

/**结构体：编码器的值**/
typedef struct
{
    // 左电机编码器的绝对值
    int64 left_encoder;
    // 右电机编码器的绝对值
    int64 right_encoder;
} RobotEncoderData;

/**
     * 结构体：表示机器人里程计数据
     **/
typedef struct
{
    // 机器人平面X位置，单位是m
    float32 x;
    // 机器人平面Y位置，单位是m
    float32 y;
    // 机器人朝向角，单位是rad，范围是-3.14～3.14
    float32 theta;
    // 四元数
    float32 qua_x;
    float32 qua_y;
    float32 qua_z;
    float32 qua_w;
    // 测量获得的机器人的线速度
    float32 linear_x;
    // 测量获得的机器人的角速度
    float32 angular_z;

} RobotPoseData;

typedef struct
{
    /*int8型 非零,有错*/
    // 心跳报文超时
    int16 heartbeat_timeout = 0;
    // 驱动器出错
    int16 drive_error = 0;
    // 前碰撞
    int16 hit_front = 0;
    // 后碰撞
    int16 hit_back = 0;
    //按钮急停
    int16 button_stop = 0;
    //无线急停
    int16 wireless_stop = 0;
    // 网络急停
    int16 net_stop = 0;
    // 电池出错
    int16 battery_error = 0;
    // 音频出错
    int16 sound_error = 0;

} HeartbeatMesssage;

/**
      * 结构体：表示机器人IMU数据
      **/
typedef struct
{
    // 角速度，单位是rad/sec
    float angular_x;
    float angular_y;
    float angular_z;
    // 加速度，单位是m/s^2
    float acc_x;
    float acc_y;
    float acc_z;
    // 角度,单位是rad
    float roll;
    float pitch;
    float yaw;

} RobotImuData;
} // namespace youi_robot_namespace

#endif
