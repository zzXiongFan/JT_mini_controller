#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>


#include <can_bus.h>

#include <linux/can/raw.h>


#define Radius = 0.5;//与controller里面的保持一致
#define Pi = 3.1415926;
//InfraredSerial infrared_serial;
canbus bus("can0");

void Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    receive_cmd = 0;
    controller.twist2motor(msg->linear.x,msg->angular.z);
//    ROS_INFO("writing %d %d",left_velocity,right_velocity);
}

void velocityCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    short left_velocity = msg->data[0];
    short right_velocity = msg->data[1];
    if(left_velocity >= 0)
    {
        left_velocity = (left_velocity*60) /(Pi*Radius);
    }
    else
    {
        left_velocity = (left_velocity*60) /(Pi*Radius);
        left_velocity = 65536 + left_velocity;
    }
    
    
    if(right_velocity >= 0)
    {
        right_velocity = (right_velocity*60) /(Pi*Radius);
    }
    else
    {
        right_velocity = (right_velocity*60) /(Pi*Radius);
        right_velocity = 65536 + right_velocity;
    }

//添加停止确认模块
    uint8_t buf[8];
    buf[0] = 0x0A;
    buf[1] = floor(left_velocity%256);
    buf[2] = floor(left_velocity/256);
    buf[3] = floor(right_velocity%256);
    buf[4] = floor(right_velocity/256);
    buf[5] = 0x00;
    buf[6] = 0x00;
    buf[7] = 0x00;

    /*
    一些速度转换的操作和buf设置的操作,还需要测试数据是否正常，现认为默认存储为补码，直接发送即可

    */
    //infrared_serial.uart_write(left_velocity,right_velocity);

    if (!bus.send(0x307, buf, sizeof(buf))) {
        
        ROS_ERROR("Write can buffer failed!!  Notice the communication timeout!!");

    }else
    {
        ROS_INFO("writing %d %d",left_velocity,right_velocity);
    }
    
    //ROS_INFO("writing %d %d",left_velocity,right_velocity);
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"mobile_base");
    ros::NodeHandle nh;
   
    if(!bus.begin()){
        ROS_ERROR("canbus begin failed");
        //return 1;
    }
    /**订阅速度函数**/ 
    ros::Subscriber velocity_sub = nh.subscribe("left_right_wheel_speed",1000,&velocityCallback);
    ros::Subscriber velocity_sub = nh.subscribe("cmd_vel_mux/input/teleop",1000,&Callback);
    /**主循环**/

    ros::spin();
    //infrared_serial.closeUart();
    bus.~canbus();
    return 0;
    
}
