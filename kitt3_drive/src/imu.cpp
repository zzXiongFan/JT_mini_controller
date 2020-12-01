#include "imu.h"

IMU::IMU()
{
    isOpened = false;

    // imuData初始化
    imuData.acc_x = 0;
    imuData.acc_y = 0;
    imuData.acc_z = 0;
    imuData.angular_x = 0;
    imuData.angular_y = 0;
    imuData.angular_z = 0;
    imuData.pitch = 0;
    imuData.roll = 0;
    imuData.yaw = 0;

    try
    {
        //初始化串口
        imu_serial.setPort(IMU_DEV);//"dev/ttyS0"
        // 设置波特率
        imu_serial.setBaudrate(IMU_PORT);
        // 设置打开超时时间
        serial::Timeout to0 = serial::Timeout::simpleTimeout(3000);
        imu_serial.setTimeout(to0);
        imu_serial.open();

        if(imu_serial.isOpen())
        {
            isOpened = true;
            cout<<"The imu serial has been opened!"<<endl;

            imuData.acc_x=0;
            imuData.acc_y=0;
            imuData.acc_z=0;
            imuData.angular_x = 0;
            imuData.angular_y = 0;
            imuData.angular_z = 0;
            imuData.pitch = 0;
            imuData.roll = 0;
            imuData.yaw = 0;

            tx_buffer[0] = 0x68;
            tx_buffer[1] = 0x04;
            tx_buffer[2] = 0x00;
            tx_buffer[3] = 0x04;
            tx_buffer[4] = 0x08;

            zero_euler[0] = 0x68;
            zero_euler[1] = 0x04;
            zero_euler[2] = 0x00;
            zero_euler[3] = 0x28;
            zero_euler[4] = 0x2c;

            while(isOpened)
            {
                int rett = resetUart(10);
                if(rett == 1)
                {
                    cout<<"Imu has been reset!"<<endl;
                    // 开启线程
                    int ret = start();
                    if(ret == 0)
                    {
                        cout<<"Create Imu pthread successfully!"<<endl;
                    }
                    break;

                }else{
                    cout<<"Imu has not been reset,so can not create new thread!"<<endl;
                }
                sleep(1);
            }

        }
    }
    catch (serial::IOException& e)
    {
        cout<<"Can not open the imu serial!"<<endl;
    }
}

IMU::~IMU()
{
    isOpened = false;
}

void* IMU::start_thread(void *arg)
{
    IMU *ptr = (IMU *)arg;
    ptr->explain_line();
}

int IMU::start()
{
    if(pthread_create(&pid_,NULL,start_thread,(void *)this) != 0) //´创建一个线程(必须是全局函数)
    {
        return -1;
    }
    return 0;
}

int8 IMU::resetUart(int8 times)
{
    for(int i = 0 ; i < times ; i++)
    {
        imu_serial.write(zero_euler,5);
        imu_serial.read(zero_euler_rx,6);
        printSerial(zero_euler_rx,6);
        // 判断是否成功清零
        if(zero_euler_rx[0] == 0x68 && zero_euler_rx[4] == 0x00)
        {
            cout<<"Imu: roll pitch yaw have cleared!"<<" "<<i<<endl;
            return 1;
        }else {
            cout<<"Error: Can not reset the imu!"<<endl;
            return 0;
        }
    }
    return 0;
}

void IMU::explain_line()
{
    unsigned char sum;
    int j = 0;
    float32 roll = 0;
    float32 pitch = 0;
    float32 yaw = 0;
    float32 groy_x = 0;
    float32 groy_y = 0;
    float32 groy_z = 0;
    float32 acc_x = 0;
    float32 acc_y = 0;
    float32 acc_z = 0;

    while (isOpened) {

        sum = 0x00;
        j = 0;
        // 串口发送读取命令
        // imu_serial.write(tx_buffer,5);
        // 判断是否收到返回数据
        imu_serial.read(rx_header,1);
        if(rx_header[0] == 0x7F)
        {
            imu_serial.read(rx_header,1);
            if(rx_header[0] == 0x80)
            {
                imu_serial.read(rx_buffer,21);
                for(int i =0;i<20;i++)
                {
                    sum = sum + rx_buffer[i];
                }
                //sum = sum + rx_header[0];
                sum = 0xFF - sum
                if(sum == rx_buffer[20])
                {
//                    printSerial(rx_buffer,30);
                    // 对字段进行解析

                    acc_x = rx_buffer[0] + rx_buffer[1] * 256;
                    acc_y = rx_buffer[2] + rx_buffer[3] * 256;
                    acc_z = rx_buffer[4] + rx_buffer[5] * 256;
                    groy_x  = rx_buffer[6] + rx_buffer[7] * 256;
                    groy_y = rx_buffer[8] + rx_buffer[9] * 256;
                    groy_z   = rx_buffer[10] + rx_buffer[11] * 256;
                    roll  = rx_buffer[12] + rx_buffer[13] * 256;
                    pitch = rx_buffer[14] + rx_buffer[15] * 256;
                    yaw   = rx_buffer[16] + rx_buffer[17] * 256;
                    temprature_imu = rx_buffer[18] + rx_buffer[19] * 256;

                    imuData.roll = roll * rate_scale / sensor_scale;
                    imuData.pitch = pitch * rate_scale / sensor_scale;
                    imuData.yaw = yaw * rate_scale / sensor_scale;
                    imuData.acc_x = acc_x * 9.8 * accel_scale / sensor_scale;
                    imuData.acc_y = acc_y * 9.8 * accel_scale / sensor_scale;
                    imuData.acc_z = acc_z * 9.8 * accel_scale / sensor_scale;
                    imuData.angular_x = groy_x  * angle_scale / sensor_scale;
                    imuData.angular_y = groy_y  * angle_scale / sensor_scale;
                    imuData.angular_z = groy_z  * angle_scale / sensor_scale;
		   // cout<<imuData.yaw *180/PI<<endl;
                    //cout<<imuData.roll<<" "<<imuData.pitch<<" "<<imuData.yaw *180/PI<<" "<<imuData.acc_x<<" "<<imuData.acc_y<<" "<<imuData.acc_z<<" "<<imuData.angular_x<<" "<<imuData.angular_y<<" "<<imuData.angular_z<<endl;
                }
            }
        }
    }
}

void IMU::printSerial(uint8_t *data, int8 len)
{
    for(int k = 0 ; k < len ; k++)
    {
        printf("%x ",data[k]);
    }
    printf("\r\n");
}

void IMU::closeUart()
{
    if(isOpened)
    {
        isOpened = false;
        sleep(1);
        imu_serial.close();
        if(!imu_serial.isOpen())
        {
            cout<<"Imu serial closed!"<<endl;
        }
    }else {
        cout<<"Imu serial has not been openned!"<<endl;
    }

}

