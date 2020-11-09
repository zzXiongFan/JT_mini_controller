#ifndef COMMAND_H
#define COMMAND_H

/**机器人运动相关的参数**/

#define DEV "/dev/youibot"
#define DEV_IMU "/dev/imu"
#define FALSE  -1
#define TRUE   0
#define NUM_THREADS 1
#define PI 3.1415926

/**机器人控制相关的参数**/

#define BASE_WIDTH 0.616
//0.422
#define TICKS_METER 49883
#define ODOM_USE_IMU 1

/**摄像头相关的参数**/

#define IMAGEWIDTH 640
#define IMAGEHEIGHT 480
#define FPS  30
#define ISSHOW 1

#endif
