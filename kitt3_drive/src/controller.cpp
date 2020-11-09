#include "controller.h"

void* Controller::start_thread(void *arg)
{
    Controller *ptr = (Controller *)arg;
    ptr->produce_odom();
}

int Controller::start()
{
    if(pthread_create(&pid,NULL,start_thread,(void *)this) != 0) //´创建一个线程(必须是全局函数)
    {
        return -1;
    }
    return 0;
}

void Controller::twist2motor(float linear_x, float angular_z)
{
    float dx = linear_x;
    float dr = angular_z;

    float right = 1.0 * dx + dr * base_width / 2.0;
    float left = 1.0 * dx - dr * base_width / 2.0;

    mot.motor_left = floor(left * 100);
    mot.motor_right = floor(right * 100);

    // cout<<"motor: "<<mot.motor_left<<" "<<mot.motor_right<<endl;
}

const long Controller::getTime()
{
    const boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    const boost::posix_time::time_duration td = now.time_of_day();
//    const long hours  = td.hours();
//    const long minutes  = td.minutes();
//    const long seconds  = td.seconds();
    const long milliseconds = td.total_microseconds();
//    cout<<milliseconds<<endl;
    return milliseconds;
}

void Controller::produce_odom()
{
    /**左右两个轮子分别走过的路程**/
    float d_left;
    float d_right;
    /**里程计数据**/
    float d = 0;
    float th = 0;
    float th_add = 0;
    float x = 0;
    float y = 0;
    /**速度**/
    float dx = 0;
    float dz = 0;
    /**左右两个轮子的编码器读数**/
    long left = 0;
    long right = 0;

    while(publish_odom)
    {

        left = encoder_.left_encoder;
        right = encoder_.right_encoder;
        /**系统时间精确到ms**/
        now = getTime();
//        cout<<"now: "<<now<<endl;
        if(now > t_next)
        {
            elapsed = now - then;
            then = now;
            if(enc_left == NULL && enc_right == NULL){
                d_left = 0;
                d_right = 0;
                //printf("null\r\n");
            }else{
                d_left = (left - enc_left)/ ticks_meter ;
                d_right = (right - enc_right)/ ticks_meter;
            }
            //cout << (left - enc_left) << " " << (right - enc_right) << endl;
            enc_left = left;
            enc_right = right;
            
            /**左右两个轮子走过的平均距离**/
            d = (d_left + d_right) / 2.0;
            /**转过的角度**/
            th = (d_right - d_left) / base_width;
            //printf("d_right:%f\n",d_left);
            //printf("d_left:%f\n",d_right);
            //printf("right:%i\n",left);
            //printf("left:%i\n",right);
            //printf("enc_right:%i\n",enc_left);
            //printf("enc_left:%i\n",enc_right);
            //printf("base_width:%f\n",base_width);

            dx = d / elapsed;
            dz = th / elapsed;
            if(d != 0)
            {
                float xx = cos(th) * d;
                float yy = -sin(th) * d;
                x = x + (cos(th_add) * xx - sin(th_add) * yy);
                y = y + (sin(th_add) * xx + cos(th_add) * yy);
            }
            if(th != 0)
            {
                th_add = th_add + th;

                if(th_add > PI)
                {
                    th_add = th_add - 2 * PI;
                }
                if (th_add < - PI) {
                    th_add = th_add + 2 * PI;
                }
            }
            //cout << "th " << th << endl;
            //cout << "d " << d << endl;
            pose_.dx = dx;
            pose_.dz = dz;
            pose_.x = x;
            pose_.y = y;
            pose_.theta = th_add;
            pose_.qua_z = sin(th_add / 2.0);
            pose_.qua_w = cos(th_add / 2.0);
            //cout<<pose_.x<<" "<<pose_.y<<" ";
            //cout<<pose_.theta<<endl;
        }
        /**睡50ms**/
        sleep(duration / 1000.0);
    }
}
void Controller::close_thread()
{
    publish_odom = false;
    cout<<publish_odom<<endl;
}
