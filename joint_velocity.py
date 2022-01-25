#!/usr/bin/env python2
import rospy
import numpy as np
from sensor_msgs.msg import Image, JointState
from math import *
from std_msgs.msg import Float64MultiArray
from kortex_driver.srv import *
from kortex_driver.msg import *
from kortex_driver.msg import Base_JointSpeeds
import kinova_jac_func_new
import Vcam_to_Base_kinova 
import Plot_task_to_joint
from scipy.spatial.transform import Rotation as RR



def jointstates_cb(js):
    global th1, th2, th3, th4, th5, th6
    global th
    global Varm
    global mom
    # th = js.position
    th1 = js.position[0]
    th2 = js.position[1]
    th3 = js.position[2]
    th4 = js.position[3]
    th5 = js.position[4]
    th6 = js.position[5]
    th = np.array([th1, th2, th3, th4, th5, th6])
    print("joint_angles = ", th)
    



def main():
    rospy.init_node('Publish_joint_vel', anonymous=True)
    pub = rospy.Publisher('/my_gen3_lite/in/joint_velocity',Base_JointSpeeds,  queue_size=10)
    rospy.Subscriber("/my_gen3_lite/joint_states",Base_JointSpeeds, jointstates_cb )
    rate = rospy.Rate(75)
    jv = Base_JointSpeeds()

    while not rospy.is_shutdown():
        
        joint_speeds = []
        msg = JointSpeed()
        msg.joint_identifier = 1
        msg.value = -5.5
        msg.duration = 0

        msg.joint_identifier = 2
        msg.value = 0.5
        msg.duration = 0

        msg.joint_identifier = 3
        msg.value = 0.5
        msg.duration = 0

        msg.joint_identifier = 4
        msg.value = 0.5
        msg.duration = 0

        msg.joint_identifier = 5
        msg.value = 0.5
        msg.duration = 0

        msg.joint_identifier = 6
        msg.value = 0.5
        msg.duration = 0
        
        joint_speeds.append(msg)

        jv.joint_speeds = joint_speeds
        jv.duration = 0

        pub.publish(jv)
        rate.sleep()
    


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


        