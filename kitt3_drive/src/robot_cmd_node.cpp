#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <motion.h>
#include <can_bus.h>

#include <linux/can/raw.h>

//InfraredSerial infrared_serial;
canbus bus("can0");

void velocityCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    int left_velocity = msg->data[0];
    int right_velocity = msg->data[1];

    uint8_t buf[8];

    buf[0] = 0x0A;
    buf[1] = left_velocity%256;
    buf[2] = left_velocity/256;
    buf[3] = right_velocity%256;
    buf[4] = right_velocity/256;
    buf[5] = 0x00;
    buf[6] = 0x00;
    buf[7] = 0x00;

    /*
    一些速度转换的操作和buf设置的操作
    uint8_t buf[8];
    buf[0] = left_velocity;
    buf[1] = right_velocity;

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

    /**主循环**/

    ros::spin();
    //infrared_serial.closeUart();
    bus.~canbus();
    return 0;
    
}
