#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <motion.h>
#include <can_bus.h>

//InfraredSerial infrared_serial;
canbus bus("can0");

IMU imu;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"mobile_base");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    /**编码器数值**/
    //ros::Publisher encoder_pub = nh.advertise<std_msgs::Int32MultiArray>("/encoder_cnts",100);
    /**电池电量输出**/
    ros::Publisher power_pub = nh.advertise<std_msgs::Float32>("/battery_Topic",100);
    /**电机速度 r/min**/
    ros::Publisher speed_pub = nh.advertise<std_msgs::Int16MultiArray>("/speed_wheel",100);
    /**电机的母线电流 ma**/
    //ros::Publisher electic_pub = nh.advertise<std_msgs::Int16MultiArray>("/electic_topic",100);
    /**发布imu数据**/
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_data",100);

    int resp = bus.start();
    
    /**主循环**/
    while (ros::ok()) {
//        cout<<ros::Time::now().nsec<<endl;
//        // 读取编码器
//        information = motion.explainLine(fd,buff);
        //std_msgs::Int32MultiArray encoder_msgs;
        //encoder_msgs.data.push_back(infrared_serial.information.left_encoder);
        //encoder_msgs.data.push_back(infrared_serial.information.right_encoder);
        //encoder_pub.publish(encoder_msgs);
        
        std_msgs::Float32 power_msgs;
        power_msgs.data = bus.information.power;
        power_pub.publish(power_msgs);
        
        std_msgs::Int16MultiArray speed_msgs;
        speed_msgs.data.push_back(bus.information.left_speed);
        speed_msgs.data.push_back(bus.information.right_speed);
        speed_pub.publish(speed_msgs);
        
        //std_msgs::Int16MultiArray electric_msgs;
        //electric_msgs.data.push_back(bus.information.left_electric);
        //electric_msgs.data.push_back(bus.information.right_electric);
        //electic_pub.publish(electric_msgs);
        // 发布IMU
        sensor_msgs::Imu imu_msg;
        std_msgs::Header h;
        h.frame_id = "base_footprint";
        h.stamp = ros::Time::now();
        imu_msg.header = h;
        imu_msg.orientation_covariance = {0,0,0,0,0,0,0,0,0};//{1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
        imu_msg.angular_velocity_covariance = {0,0,0,0,0,0,0,0,0};//{1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
        imu_msg.linear_acceleration_covariance = {0,0,0,0,0,0,0,0,0};//{-1,0,0,0,0,0,0,0,0};
        imu_msg.angular_velocity.x = imu.imuData.angular_x;
        imu_msg.angular_velocity.y = imu.imuData.angular_y;
        imu_msg.angular_velocity.z = imu.imuData.angular_z;
        imu_msg.linear_acceleration.x = imu.imuData.acc_x;
        imu_msg.linear_acceleration.y = imu.imuData.acc_y;
        imu_msg.linear_acceleration.z = imu.imuData.acc_z;

        tf::Quaternion q = tf::createQuaternionFromRPY(imu.imuData.roll, imu.imuData.pitch, imu.imuData.yaw);
        q.normalize();
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();
//        cout<<imu.imu_.roll<<" "<<imu.imu_.pitch<<" "<<imu.imu_.yaw<<endl;

        imu_pub.publish(imu_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    infrared_serial.closeUart();
    imu.closeUart();
    return 0;
}
