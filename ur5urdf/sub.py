#! /usr/bin/env python3

from numpy.lib.financial import rate
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


pub1_right = rospy.Publisher('ur5urdf/joint_1_controller/command', Float64, queue_size=10) # Add your topic here between ''. Eg '/my_robot/steering_controller/command'
pub1_left = rospy.Publisher('ur5urdf/joint_1_controller/command', Float64, queue_size=10)

pub2_right = rospy.Publisher('ur5urdf/joint_2_controller/command', Float64, queue_size=10) # Add your topic here between ''. Eg '/my_robot/steering_controller/command'
pub2_left = rospy.Publisher('ur5urdf/joint_2_controller/command', Float64, queue_size=10)

pub3_right = rospy.Publisher('ur5urdf/joint_3_controller/command', Float64, queue_size=10) # Add your topic here between ''. Eg '/my_robot/steering_controller/command'
pub3_left = rospy.Publisher('ur5urdf/joint_3_controller/command', Float64, queue_size=10)

pub4_right = rospy.Publisher('ur5urdf/joint_4_controller/command', Float64, queue_size=10) # Add your topic here between ''. Eg '/my_robot/steering_controller/command'
pub4_left = rospy.Publisher('ur5urdf/joint_4_controller/command', Float64, queue_size=10)

pub5_right = rospy.Publisher('ur5urdf/joint_5_controller/command', Float64, queue_size=10) # Add your topic here between ''. Eg '/my_robot/steering_controller/command'
pub5_left = rospy.Publisher('ur5urdf/joint_5_controller/command', Float64, queue_size=10)

pub6_right = rospy.Publisher('ur5urdf/joint_6_controller/command', Float64, queue_size=10) # Add your topic here between ''. Eg '/my_robot/steering_controller/command'
pub6_left = rospy.Publisher('ur5urdf/joint_6_controller/command', Float64, queue_size=10)

 
twist = Float64()


def turn_push(data):
    rospy.loginfo(rospy.get_caller_id() + "data received %f", data.data[1])

    #### Initial Values to set the pose of robot
    twist.data = 0 # 0 Degrees
    pub1_right.publish(twist)
    pub1_left.publish(twist)

    twist.data = 1.309 # 75 Degrees
    pub2_right.publish(twist)
    pub2_left.publish(twist)

    twist.data = 0 # 0 Degrees
    pub3_right.publish(twist)
    pub3_left.publish(twist)

    twist.data = -1.0472 # -60 Degrees
    pub4_right.publish(twist)
    pub4_left.publish(twist)

    twist.data = 1.0472 # 60 Degrees
    pub5_right.publish(twist)
    pub5_left.publish(twist)

    twist.data = 0.785398 # 45 Degrees
    pub6_right.publish(twist)
    pub6_left.publish(twist)
    

    #### Angels from the publisher
    pub1_right.publish(data.data[0])
    pub1_left.publish(data.data[0])
    
    pub2_right.publish(data.data[1])
    pub2_left.publish(data.data[1])

    pub3_right.publish(data.data[3])
    pub3_left.publish(data.data[2])

    pub4_right.publish(data.data[3])
    pub4_left.publish(data.data[3])

    pub5_right.publish(data.data[4])
    pub5_left.publish(data.data[4])

    pub6_right.publish(data.data[5])
    pub6_left.publish(data.data[5])
    
    
def listener():
 
    rospy.init_node('ur5', anonymous=True)

    rospy.Subscriber("turn", Float64MultiArray, turn_push)

    rospy.spin()
 
if __name__ == '__main__':
    listener()
