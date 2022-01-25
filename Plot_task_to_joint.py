#!/usr/bin/env python2
import time
import math as m
from math import pi
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc

Varm0 = []
Varm1 = []
Varm2 = []
Varm3 = []
Varm4 = []
Varm5 = []

M = []

th0 = []
th1 = []
th2 = []
th3 = []
th4 = []
th5 = []

X=[]
Y=[]
Z=[]
R_x=[]
R_y=[]
R_z=[]

Er0=[]
Er1=[]
Er2=[]
Er3=[]
Er4=[]
Er5=[]

Er_N=[]

time1 = []
i = 0
initial_time=time.time()

def plot(Varm,mom,th,EE_pos,Euler_ang,N,Err_tn,Err_Rn):
    global i
    global Varm0,Varm1,Varm2,Varm3,Varm4,Varm5
    global th0,th1,th2,th3,th4,th5
    global X,Y,Z,R_x,R_y,R_z
    global time1
    global M
    global Er0,Er1,Er2,Er3,Er4,Er5
    global Er_N

    M.append(mom)

    Er_N.append(N)

    th0.append(th.item(0)*180/pi)
    th1.append(th.item(1)*180/pi)
    th2.append(th.item(2)*180/pi)
    th3.append(th.item(3)*180/pi)
    th4.append(th.item(4)*180/pi)
    th5.append(th.item(5)*180/pi)

    Varm0.append(Varm.item(0))
    Varm1.append(Varm.item(1))
    Varm2.append(Varm.item(2))
    Varm3.append(Varm.item(3))
    Varm4.append(Varm.item(4))
    Varm5.append(Varm.item(5))

    X.append(EE_pos.item(0))
    Y.append(EE_pos.item(1))
    Z.append(EE_pos.item(2))
    R_x.append(Euler_ang.item(0)*180/pi)
    R_y.append(Euler_ang.item(1)*180/pi)
    R_z.append(Euler_ang.item(2)*180/pi)

    Er0.append(Err_tn.item(0))
    Er1.append(Err_tn.item(1))
    Er2.append(Err_tn.item(2))
    Er3.append(Err_Rn.item(0))
    Er4.append(Err_Rn.item(1))
    Er5.append(Err_Rn.item(2))


    time1.append(time.time()-initial_time)

    if i == 600:
        
        ##----Joint_velocity------##
        fig1 =  plt.figure()
        plt.plot(time1,Varm0, 'r',ls='solid', label='$\dot \Theta_1 $')
        plt.plot(time1,Varm1, 'g', label='$\dot \Theta_2 $')
        plt.plot(time1,Varm2, 'b', label='$ \dot \Theta_3 $')
        plt.plot(time1,Varm3, 'c', label='$ \dot \Theta_4 $')
        plt.plot(time1,Varm4, 'y', label='$ \dot \Theta_5 $')
        plt.plot(time1,Varm5, 'k', label='$ \dot \Theta_6 $')
        plt.xlabel('Time (s)')
        plt.ylabel('Joint_Velocity')
        plt.legend(frameon=False)
        plt.autoscale(enable=True, axis = 'both',tight = None)
        plt.savefig('Joint_Velocity.png', transperent=True)

        ##----Measure_of_Singularity------##
        fig2 = plt.figure()
        plt.plot(time1,M,'b', label= 'Measure_of_Singularity')
        plt.xlabel('Time (s)')
        plt.ylabel('mom')
        plt.savefig('Singularity.png', transperent=True)

        ##------Joint_Angles--------------##
        fig3 = plt.figure()
        plt.plot(time1,th0,'r', label='$\Theta_1$')
        plt.plot(time1,th1,'g', label='$\Theta_2$')
        plt.plot(time1,th2,'b', label='$\Theta_3$')
        plt.plot(time1,th3,'c', label='$\Theta_4$')
        plt.plot(time1,th4,'y', label='$\Theta_5$')
        plt.plot(time1,th5,'k', label='$\Theta_6$')
        plt.xlabel('Time (s)')
        plt.ylabel('Joint_Angle (rad)')
        plt.legend(frameon=False)
        plt.savefig('Joint_Ang.png', transperent=True)

        ##--------End_effector_position----------##
        ##--------Using Forward Kinematics--------##
        fig4 = plt.figure()
        plt.plot(time1,X,'r', label='$ x $')
        plt.plot(time1,Y,'g', label='$ y $')
        plt.plot(time1,Z,'b', label='$ z $')
        plt.xlabel('Time (s)')
        plt.ylabel('End_effector_pose')
        plt.legend(frameon=False)
        plt.savefig('End_effector_pose.png', transperent=True)

        fig5 = plt.figure()
        plt.plot(time1,R_x,'c', label='$ A $')
        plt.plot(time1,R_y,'y', label='$ B $')
        plt.plot(time1,R_z,'k', label='$ C $')
        plt.xlabel('Time (s)')
        plt.ylabel('End_effector_orientation (rad)')
        plt.legend(frameon=False)
        plt.savefig('End_effector_orr.png', transperent=True)

        fig6 = plt.figure()
        plt.plot(time1,Er0,'r', label='$Ex$')
        plt.plot(time1,Er1,'g', label='$Ey$')
        plt.plot(time1,Er2,'b', label='$Ez$')
        plt.plot(time1,Er3,'c', label='$Erx$')
        plt.plot(time1,Er4,'y', label='$Ery$')
        plt.plot(time1,Er5,'k', label='$Erz$')
        plt.xlabel('Time (s)')
        plt.ylabel('Error')
        plt.legend(frameon=False)
        plt.savefig('Err_vec.png', transperent=True)

        ##----Measure_of_Singularity------##
        fig7 = plt.figure()
        plt.plot(time1,Er_N,'b', label= 'Norm of Error')
        plt.xlabel('Time (s)')
        plt.ylabel('Norm_E')
        plt.savefig('norm_error.png', transperent=True)

        print('SAVED')

    i = i+1