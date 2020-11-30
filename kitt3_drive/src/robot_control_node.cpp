
/*
 * Bug: filter time older than odom message buffer
*/

#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <controller.h>
#include <tf/transform_broadcaster.h>

using namespace std;

Controller controller;
/**判断是否收到速度命令**/
int receive_cmd = 0;

float x,y,z,w;

/*void Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    receive_cmd = 0;
    controller.twist2motor(msg->linear.x,msg->angular.z);
//    ROS_INFO("writing %d %d",left_velocity,right_velocity);
}*/

void speedCallback(const std_msgs::Int16MultiArray::ConstPtr& msg){
//    controller.publish_odom = true;
    controller.encoder_.left_speed = msg->data[0];
    controller.encoder_.right_speed = msg->data[1];
    /*
        如果发送直线速度和转弯速度，在此处添加twist_to_motor
    */

}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    x = msg->orientation.x;
    y = msg->orientation.y;
    z = msg->orientation.z;
    w = msg->orientation.w;
//    cout<<ODOM_USE_IMU<<endl;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"controller_base");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);//looprate 要和 produce odom中的duration基本一致
    /**发布速度命令**/
    //ros::Publisher velocity_pub = nh.advertise<std_msgs::Int16MultiArray>("/left_right_wheel_speed",100);
    /**发布里程计命令**/
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom",1000);
    /**订阅速度函数**/
    //ros::Subscriber velocity_sub = nh.subscribe("cmd_vel_mux/input/teleop",1000,&Callback);//改在robot_cmd_node内部完成
    /**订阅编码器函数**/
    ros::Subscriber encoder_sub = nh.subscribe("speed_wheel",1000,&speedCallback);

    ros::Subscriber imu_sub = nh.subscribe("/imu_data",1000,&imuCallback);
    std_msgs::Int16MultiArray msg_array;
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion qua;

    while(ros::ok())
    {
        ros::spinOnce();
        /**发布速度命令,转移到robot_cmd_node**/
        /*if(receive_cmd < 3){
            msg_array.data.clear();
            msg_array.data.push_back(controller.mot.motor_left);
            msg_array.data.push_back(controller.mot.motor_right);
            velocity_pub.publish(msg_array);
            receive_cmd++;
        }else {
            msg_array.data.clear();
            msg_array.data.push_back(0);
            msg_array.data.push_back(0);
            velocity_pub.publish(msg_array);
            receive_cmd++;
        }
        if(receive_cmd > 1000){
            receive_cmd = 100;
        }*/
        /**发布里程计数据**/
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = controller.pose_.x;
        odom.pose.pose.position.y = controller.pose_.y;
        odom.pose.pose.position.z = 0;
        if(ODOM_USE_IMU > 0)
        {
            qua.x = x;
            qua.y = y;
            qua.z = z;
            qua.w = w;
        }else{
            qua.x = 0;
            qua.y = 0;
            qua.z = controller.pose_.qua_z;
            qua.w = controller.pose_.qua_w;
        }
        odom.pose.pose.orientation = qua;

        odom.twist.twist.linear.x = controller.pose_.dx;
        odom.twist.twist.angular.z = controller.pose_.dz;
        if(controller.pose_.dx != 0 && controller.pose_.dz != 0)
        {
            odom.pose.covariance = {1e-3, 0, 0, 0, 0, 0,
                                    0, 1e-3, 0, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e3};
            odom.twist.covariance = {1e-3, 0, 0, 0, 0, 0,
                                    0, 1e-3, 0, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e3};
        }else{
            odom.pose.covariance =  {1e-9, 0, 0, 0, 0, 0,
                                     0, 1e-3, 1e-9, 0, 0, 0,
                                     0, 0, 1e6, 0, 0, 0,
                                     0, 0, 0, 1e6, 0, 0,
                                     0, 0, 0, 0, 1e6, 0,
                                     0, 0, 0, 0, 0, 1e-9};
            odom.twist.covariance =  {1e-9, 0, 0, 0, 0, 0,
                                      0, 1e-3, 1e-9, 0, 0, 0,
                                      0, 0, 1e6, 0, 0, 0,
                                      0, 0, 0, 1e6, 0, 0,
                                      0, 0, 0, 0, 1e6, 0,
                                      0, 0, 0, 0, 0, 1e-9};
        }
        odom_pub.publish(odom);

        // tf::Transform transform;
        // static tf::TransformBroadcaster br;
        // transform.setOrigin(tf::Vector3(controller.pose_.x,controller.pose_.y,0.0));
        // tf::Quaternion q;
        // q.setRPY(0,0,controller.pose_.theta);
        // transform.setRotation(q);
        // br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"base_footprint","odom"));

        loop_rate.sleep();
    }
    controller.close_thread();
    return 0 ;
}
