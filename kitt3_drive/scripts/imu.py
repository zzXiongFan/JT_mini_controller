#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import tf

def ImuCallback(msg):
    (r,p,y)=tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    print(y)
def IMUEuler():
     # In ROS, nodes are uniquely named. If two nodes with the same
     # node are launched, the previous one is kicked off. The
     # anonymous=True flag means that rospy will choose a unique
     # name for our 'listener' node so that multiple listeners can
     # run simultaneously.
     rospy.init_node('listener', anonymous=True)

     rospy.Subscriber("imu_data", Imu, ImuCallback)

     # spin() simply keeps python from exiting until this node is stopped
     rospy.spin()

if __name__ == '__main__':
    IMUEuler()