#!/usr/bin/env python
# -*- coding: UTF-8 -*-

'''
2018-06-02 : 将/odom_combained话题消息转化成Odometry类型
'''

import rospy
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Twist , PoseWithCovarianceStamped , TwistWithCovariance
from nav_msgs.msg import Odometry

class Transform():
    def __init__(self):
        rospy.init_node('transform_to_odometry',anonymous=True)
        self.rate = 30.0
        # 订阅robot_pose_ekf节点滤波后的里程计信息
        rospy.Subscriber('odom_combined', PoseWithCovarianceStamped, self.odomCallback)
        # 订阅odom话题的消息
        rospy.Subscriber('odom',Odometry,self.callback)
        # 发布携带Odometry消息类型的里程计信息
        self.odomPub = rospy.Publisher("odom_ekf", Odometry, queue_size=10)
        self.twistwithcovariance = TwistWithCovariance()
        # 发布坐标变换
        self.br = TransformBroadcaster()
        rospy.spin()

    def callback(self,msg):

        self.twistwithcovariance = msg.twist

    def odomCallback(self,msg):
        self.odom = Odometry()
        self.odom.pose = msg.pose
        self.odom.header = msg.header
        self.odom.header.frame_id = "odom_combined"
        self.odom.child_frame_id = "base_footprint"
        self.odom.twist = self.twistwithcovariance
        # 发布坐标变换
        self.br.sendTransform((msg.pose.pose.position.x , msg.pose.pose.position.y , 0),
                              (msg.pose.pose.orientation.x , msg.pose.pose.orientation.y , msg.pose.pose.orientation.z , msg.pose.pose.orientation.w),
                              rospy.Time.now(),
                              "base_footprint",
                              "odom_combined"
                              )
        self.odomPub.publish(self.odom)

if __name__ == '__main__':

    transform = Transform()
