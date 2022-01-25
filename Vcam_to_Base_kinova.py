
import numpy as np
from math import *
# import math as m
from math import pi


# d = np.array([0.2433, 0.030, 0.020, 0.245, 0.057, 0.105])
d = np.array([0.2433, 0.030, 0.020, 0.245, 0.057, 0.235])
a = np.array([0,0.280,0,0,0,0])
alp = np.array([pi/2, pi, pi/2, pi/2, pi/2, 0])
global theta, Rot_mat
theta=np.array([0.0,0.0,0.0,0.0,0.0,0.0])
Rot_mat=np.zeros((3,3), dtype=float)
def Vc_2_Base(Vcamera,th):
    i1 = 0
    global theta
    theta=np.array([th[0], th[1]+pi/2, th[2]+pi/2, th[3]+pi/2 , th[4]+pi, th[5]+pi/2 ])
    def trunc(values, decs=0):
        return np.trunc(values*10**decs)/(10**decs)
    theta=trunc(theta,4)
    # theta=np.array([th[0], th[1], th[2], th[3] , th[4], th[5]])
    # theta[0] = th[0]
    # theta[1]= th[1]-3.142/2
    # theta[2]=th[2]-3.142/2
    # theta[3]=th[3]-3.142/2
    # theta[4]=th[4]-3.142
    # theta[5]=th[5]-3.142/2

    print("theta=",theta)
    T = np.identity(4)
    trans = np.identity(4)
    for i1 in range (0,6):
        T = np.array([[cos(theta[i1]) , -sin(theta[i1])*cos(alp[i1]) , sin(theta[i1])*sin(alp[i1]) , a[i1]*cos(theta[i1])] ,
                    [sin(theta[i1]) ,cos(theta[i1])*cos(alp[i1]) , -cos(theta[i1])*sin(alp[i1]) , a[i1]*sin(theta[i1])] , 
                    [0 , sin(alp[i1]) , cos(alp[i1]) , d[i1]] ,
                    [0 , 0 , 0 , 1]])
        
        trans = np.matmul(trans,T)
        
    Rot_mat = np.array([[trans.item(0,0), trans.item(0,1), trans.item(0,2)], [trans.item(1,0), trans.item(1,1), trans.item(1,2)], [trans.item(2,0), trans.item(2,1), trans.item(2,2)]])
    # Rot_mat=Rot_mat.round(2)
    Rot_mat=trunc(Rot_mat,2)
    # print("Rotation matrix of Eeff ",Rot_mat)
    Rot = np.linalg.inv(Rot_mat)
    # Rot=Rot.round(2)
    Rot=trunc(Rot,2)

    ##--New Addition---##
    EE_pos = np.array([trans.item(0,3),trans.item(1,3),trans.item(2,3)])
    print("End-eff position = ",EE_pos)
    ##--##

    # print("camera velocity=", Vcamera)
    V1 = np.array([Vcamera[0], Vcamera[1], Vcamera[2]])
    V2 = np.array([Vcamera[3], Vcamera[4], Vcamera[5]])
    
    V11 = np.matmul(Rot,V1)
    V22 = np.matmul(Rot,V2)
    
    R_y = np.array([[0,0,1],[0,1,0],[-1,0,0]])
    V11=np.matmul(R_y,V11)
    V22 = 1.5*np.matmul(R_y,V2)

    Vbase = np.array([V11[0], V11[1],  V11[2], V22[0], V22[1], V22[2]]) 

    return Vbase, EE_pos, Rot_mat