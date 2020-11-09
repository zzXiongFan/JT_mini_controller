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
        imu_serial.setPort(IMU_DEV);
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
        imu_serial.write(tx_buffer,5);
        // 判断是否收到返回数据
        imu_serial.read(rx_header,1);
        if(rx_header[0] == 0x68)
        {
            imu_serial.read(rx_header,1);
            if(rx_header[0] == 0x1F)
            {
                imu_serial.read(rx_buffer,30);
                for(int i =0;i<29;i++)
                {
                    sum = sum + rx_buffer[i];
                }
                sum = sum + rx_header[0];
                if(sum == rx_buffer[29])
                {
//                    printSerial(rx_buffer,30);
                    // 对字段进行解析
                    for(int i=0;i<27;i++)
                    {
                        j = 2 * i;
                        imu_buffer[j] = (rx_buffer[2+i] >> 4) - 0x00;
                        imu_buffer[j+1] = (rx_buffer[2+i] & 0x0f) - 0x00;
                    }
                    roll = imu_buffer[1] * 100 + imu_buffer[2] * 10 + imu_buffer[3] + imu_buffer[4] * 0.1 + imu_buffer[5] * 0.01;
                    if(imu_buffer[0] == 1)
                    {
                        roll = - roll;
                    }
                    pitch = imu_buffer[7] * 100 + imu_buffer[8] * 10 + imu_buffer[9] + imu_buffer[10] * 0.1 + imu_buffer[11] * 0.01;
                    if(imu_buffer[6] == 1)
                    {
                        pitch = - pitch;
                    }
                    yaw = imu_buffer[13] * 100 + imu_buffer[14] * 10 + imu_buffer[15] + imu_buffer[16] * 0.1 + imu_buffer[17] * 0.01;
                    if(imu_buffer[12] == 1)
                    {
                        yaw = - yaw;
                    }
                    acc_x = imu_buffer[19] * 10 + imu_buffer[20] + imu_buffer[21] * 0.1 + imu_buffer[22] * 0.01 + imu_buffer[23] * 0.001;
                    if(imu_buffer[18] == 1)
                    {
                        acc_x = - acc_x;
                    }
                    acc_y = imu_buffer[25] * 10 + imu_buffer[26] + imu_buffer[27] * 0.1 + imu_buffer[28] * 0.01 + imu_buffer[29] * 0.001;
                    if(imu_buffer[24] == 1)
                    {
                        acc_y = - acc_y;
                    }
                    acc_z = imu_buffer[31] * 10 + imu_buffer[32] + imu_buffer[33] * 0.1 + imu_buffer[34] * 0.01 + imu_buffer[35] * 0.001;
                    if(imu_buffer[30] == 1)
                    {
                        acc_z = - acc_z;
                    }
                    groy_x = imu_buffer[37] * 100 + imu_buffer[38] * 10 + imu_buffer[39] + imu_buffer[40] * 0.1 + imu_buffer[41] * 0.01;
                    if(imu_buffer[36] == 1)
                    {
                        groy_x = - groy_x;
                    }
                    groy_y = imu_buffer[43] * 100 + imu_buffer[44] * 10 + imu_buffer[45] + imu_buffer[46] * 0.1 + imu_buffer[47] * 0.01;
                    if(imu_buffer[42] == 1)
                    {
                        groy_y = - groy_y;
                    }
                    groy_z = imu_buffer[49] * 100 + imu_buffer[50] * 10 + imu_buffer[51] + imu_buffer[52] * 0.1 + imu_buffer[53] * 0.01;
                    if(imu_buffer[48] == 1)
                    {
                        groy_z = - groy_z;
                    }
                    imuData.roll = roll * PI / 180.0;
                    imuData.pitch = pitch * PI / 180.0;
                    imuData.yaw = yaw * PI / 180.0;
                    imuData.acc_x = acc_x * 9.8;
                    imuData.acc_y = acc_y * 9.8;
                    imuData.acc_z = acc_z * 9.8;
                    imuData.angular_x = groy_x * PI / 180.0;
                    imuData.angular_y = groy_y * PI / 180.0;
                    imuData.angular_z = groy_z * PI / 180.0;
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

