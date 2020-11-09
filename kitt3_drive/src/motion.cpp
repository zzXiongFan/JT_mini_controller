#include <motion.h>

void* InfraredSerial::start_thread(void *arg)
{
    InfraredSerial *ptr = (InfraredSerial *)arg;
    ptr->uart_explain();
}

int InfraredSerial::start()
{
    if(pthread_create(&pid,NULL,start_thread,(void *)this) != 0) //´创建一个线程(必须是全局函数)
    {
        return -1;
    }
    return 0;
}

int InfraredSerial::uart_send(char *data, ssize_t datalen)
{
    ssize_t len = 0;
    len = write(serial_fd, data, datalen);//实际写入的长度
    if(len == datalen) {
        return len;
    } else {
        tcflush(serial_fd, TCOFLUSH);//TCOFLUSH刷新写入的数据但不传送
        return -1;
    }
}

int InfraredSerial::uart_recv(unsigned char *data, int datalen)
{
    int len=0, ret = 0;
    fd_set fs_read;
    struct timeval tv_timeout;

    FD_ZERO(&fs_read);
    FD_SET(serial_fd, &fs_read);
    tv_timeout.tv_sec  = 10;
    tv_timeout.tv_usec = 0;

    ret = select(serial_fd+1, &fs_read, NULL, NULL, &tv_timeout);
//    printf("ret = %d\n", ret);
    //如果返回0，代表在描述符状态改变前已超过timeout时间,错误返回-1

    if (FD_ISSET(serial_fd, &fs_read)) {
        len = read(serial_fd, data, datalen);
//        printf("len = %d\n", len);
        return len;
    } else {
        perror("select");
        return -1;
    }
}

void InfraredSerial::uart_write(int left, int right)
{
    char buff1[8];
    buff1[0] = 0x68;
    buff1[1] = 0x50;
    if(left < 0)
    {
        buff1[2] = 0x00;
    }else {
        buff1[2] = 0x01;
    }
    if(right < 0)
    {
        buff1[4] = 0x00;
    }else {
        buff1[4] = 0x01;
    }
    buff1[3] = (abs(left) % 256) & 0xFF;
    buff1[5] = (abs(right) % 256) & 0xFF;
    buff1[6] = 0x0d;
    buff1[7] = 0x0a;
//    printf("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \r\n",buff1[0],buff1[1],buff1[2],buff1[3],buff1[4],buff1[5],buff1[6],buff1[7]);
    int n = uart_send(buff1,8);
    if( n != 8){
        cout<<"error"<<endl;
    }
}

void InfraredSerial::printSerial(unsigned char data[], int len)
{
    for(int k = 0 ; k < len ; k++)
    {
        printf("%x ",data[k]);
    }
    printf("\r\n");
}

void InfraredSerial::uart_explain()
{

    unsigned char data[2] = {0};
    int ret;
    unsigned char buff[23] = {0};
    unsigned char sum = 0x00;
    int left_encoder = 0;
    int right_encoder = 0;
    int left_current = 0;
    int right_current = 0;
    int left_speed = 0;
    int right_speed = 0;
    int voltage = 0;

    while(isOpenned)
    {
        sum = 0x00;
        ret = uart_recv(data,1);
        if(-1 == ret)
        {
            printf("read error \n");
        }else if(data[0] == 0x68)
        {
            ret = uart_recv(data,1);
            if(-1 == ret)
            {
                printf("read error \n");
            }else if (data[0] == 0x24) {
                ret = uart_recv(buff,23);
                if(-1 == ret)
                {
                    printf("read error \n");
                }else{
                    for(int i=0;i<22;i++)
                    {
                        sum = sum + buff[i];
                    }
//                    printf("%x \n",sum);
//                    printSerial(buff,23);
                    if(sum == buff[22])
                    {
//                        printf("right data \n");
                        left_encoder = buff[4] + buff[3] * 256 + buff[2]*256*256 + buff[1]*256*256*256;
                        right_encoder = buff[9] + buff[8] * 256 + buff[7]*256*256 + buff[6]*256*256*256;
                        if(buff[0] == 0x00)
                        {
                            left_encoder = - left_encoder;
                        }
                        if(buff[5] == 0x00)
                        {
                            right_encoder = - right_encoder;
                        }
                        left_speed = buff[12] + buff[11] * 256;
                        right_speed = buff[15] + buff[14] * 256;
                        if(buff[10] == 0x00)
                        {
                            left_speed = - left_speed;
                        }
                        if(buff[13] == 0x00)
                        {
                            right_speed = - right_speed;
                        }
                        left_current = buff[18] + buff[17] * 256;
                        right_current = buff[21] + buff[20] * 256;
                        if(buff[16] == 0x00)
                        {
                            left_current = - left_current;
                        }
                        if(buff[19] == 0x00)
                        {
                            right_current = - right_current;
                        }
                        if(first_left_encoder == 0 && first_right_encoder == 0){
                            first_left_encoder = left_encoder;
                            first_right_encoder = right_encoder;
                        }
                        information.left_encoder = left_encoder - first_left_encoder;
                        information.right_encoder = right_encoder - first_right_encoder;
                        information.left_electric = left_current;
                        information.right_electric = right_current;
                        information.left_speed = left_speed;
                        information.right_speed = right_speed;
//                        cout<<information.left_encoder<<" "<<information.right_encoder<<endl;
//                        cout<<information.left_encoder<<" "<<information.right_encoder<<" "<<information.left_speed<<" "<<information.right_speed
//                           <<" "<<information.left_electric<<" "<<information.right_electric<<endl;
                    }
                }
            }

        }
    }
}

void InfraredSerial::closeUart()
{
    if(close(serial_fd) < 0){
        perror("Close Motion serial failed!");
    }else{
        printf("Close Motion serial successfully!");
    }
}

/*********************************************************************
 ***********************IMU CLASS*************************************
 ********************************************************************/


