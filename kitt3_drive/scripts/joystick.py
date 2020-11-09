#!/usr/bin/env python
# -*- coding: UTF-8 -*-

'''
20180530 10:49更新档案： 在spin函数中增加了判断两次速度都是0的时候不下发速度命令
'''
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8

class Joystick():
    def __init__(self):
        rospy.init_node('Joystick',anonymous=True)

        self.rate = rospy.get_param('~rate_joy',30)
        self.lmt_x = rospy.get_param('~lmt_x',0.1)
        self.lmt_z = rospy.get_param('~lmt_z',0.2)
        self.max_x = rospy.get_param('~max_x', 0.5)
        self.max_z = rospy.get_param('~max_z', 0.5)

        self.joy_x = 0
        self.joy_z = 0

        self.speed_x = 0
        self.speed_z = 0

        # 上一个点的速度命令
        self.last_speed_x = 0
        self.last_speed_z = 0

        self.pause = 0

        self.start = 0

        # 是否刹车标志位
        self.stop = False

        self.topic = 'cmd_vel_mux/input/teleop'  # cmd_vel_mux/input/teleop

        self.pub = rospy.Publisher(self.topic, Twist, queue_size=100)

        self.pause_arm = rospy.Publisher("arm_pause_topic", Int8 , queue_size=100)

        self.start_arm = rospy.Publisher("arm_start_topic", Int8 , queue_size=100)

        rospy.Subscriber('joy', Joy, self.callback)

    def callback(self,data):

        if (data.buttons[5] == 1):

            self.joy_x = self.max_x * data.axes[1]

            self.joy_z = self.max_z * data.axes[0]

            if abs(self.joy_x) < self.lmt_x:

                self.joy_x = 0

            if abs(self.joy_z) < self.lmt_z:

                self.joy_z = 0

            if data.buttons[3] == 1:

                self.stop = True


        else:

            self.stop = False

            self.joy_x = 0

            self.joy_z = 0

            self.move_status = False

        if(data.buttons[6] == 1 and data.buttons[7] == 1):

            self.pause = 1

            self.start = 0

        elif(data.buttons[4] == 1 and data.buttons[5] == 1):

            self.pause = 0

            self.start = 1

        else:

            self.pause = 0

            self.start = 0


        if(data.buttons[6] == 1 and data.buttons[7] == 1):

            self.pause = 1

            self.start = 0

        elif(data.buttons[4] == 1 and data.buttons[5] == 1):

            self.pause = 0

            self.start = 1

        else:

            self.pause = 0

            self.start = 0

    def spin(self):

        r = rospy.Rate(self.rate)

        while not rospy.is_shutdown():

            if self.joy_x > self.speed_x:

                self.speed_x = min(self.joy_x, self.speed_x + 0.1)

            elif self.joy_x < self.speed_x:

                self.speed_x = max(self.joy_x, self.speed_x - 0.1)

            else:

                self.speed_x = self.joy_x

            if self.joy_z > self.speed_z:

                self.speed_z = min(self.joy_z, self.speed_z + 0.1)

            elif self.joy_z < self.speed_z:

                self.speed_z = max(self.joy_z, self.speed_z - 0.1)

            else:

                self.speed_z = self.joy_z

            if self.stop == True:

                self.speed_z = 0

                self.speed_x = 0

            if self.speed_z == 0 and self.last_speed_z == 0 and self.speed_x == 0 and self.last_speed_x == 0:

                pass

            else:

                twist = Twist()

                twist.linear.x = self.speed_x

                twist.angular.z = self.speed_z

                self.pub.publish(twist)

                self.last_speed_x = self.speed_x

                self.last_speed_z = self.speed_z

            r.sleep()

if __name__ == '__main__':

    joystick = Joystick()

    joystick.spin()
