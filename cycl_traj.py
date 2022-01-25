from os import wait
import sys
import copy
from moveit_commander import move_group
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
from math import sin
import pandas as pd

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_commander',anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "orange_arm"
move_group  = moveit_commander.MoveGroupCommander(group_name)


display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

#### get basic information of robot

plan_frame = move_group.get_planning_frame()
print('planning_frame  =',plan_frame)

eff_link = move_group.get_end_effector_link()

group_names = robot.get_group_names()
#print('Group Names  =',group_names)

current_state = robot.get_current_state()
#print('current state   =',current_state)

while not rospy.is_shutdown():
        
    joint_goals = move_group.get_current_joint_values()
    
    t = 0.0    #initial time
    T = 5.0    #final time
    initial_pos = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    final_pos = (1.0,1.0,1.0,1.0,1.0,1.0)
    
    pose_list =[]
    time_list=[]
    
    while t < T:
	
	#Clearing the targets is good practice
	move_group.clear_pose_targets()

	
        #To follow cycloidal tarjectory
        joint_goals[0] = initial_pos[0] + ((final_pos[0]-initial_pos[0])/T)*(t - (T*sin(2*pi*t/T))/(2*pi))
        joint_goals[1] = initial_pos[1] + ((final_pos[1]-initial_pos[1])/T)*(t - (T*sin(2*pi*t/T))/(2*pi))
        joint_goals[2] = initial_pos[2] + ((final_pos[2]-initial_pos[2])/T)*(t - (T*sin(2*pi*t/T))/(2*pi))
        joint_goals[3] = initial_pos[3] + ((final_pos[3]-initial_pos[3])/T)*(t - (T*sin(2*pi*t/T))/(2*pi))
        joint_goals[4] = initial_pos[4] + ((final_pos[4]-initial_pos[4])/T)*(t - (T*sin(2*pi*t/T))/(2*pi))
        joint_goals[5] = initial_pos[5] + ((final_pos[5]-initial_pos[5])/T)*(t - (T*sin(2*pi*t/T))/(2*pi))
        

        #print('New goal generated')
        #print(joint_goals)
        
        current_state = robot.get_current_state()
        pose_list.append(pose_to_list(current_state.pose))
        time_list.append(t)

        move_group.go(joint_goals,wait=True)
        t = t + 0.1

        #move_group.stop()
      #  print('Goal Reached')
        rospy.sleep(0.1)
    
    move_group.stop()
    df  = pd.DataFrame()
    df['time'] = time_list
    df['data'] = pose_list
    df.to_excel('ISRO_arm_cycloidal.xlsx',index = False)
    


    
    
    
    