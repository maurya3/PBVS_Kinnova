
import numpy as np
from math import *

# d1 = 0.2433
# d2 = 0.030
# d3 = 0.020
# d4 = 0.245
# d5 = 0.057
# d6 = 0.105

# a1 = 0
a2 = 0.280
# a3 = 0
# a4 = 0
# a5 = 0
# a6 = 0


def calc_jack(th):

    th1=th[0]
    th2=th[1]   
    th3=th[2]
    th4=th[3]
    th5=th[4]
    th6=th[5]

##---Jacobian calculated using Suril sir's Jacobian func
    ## alpha= [pi/2 pi pi/2 pi/2 pi/2 0]
    ## a=[0 0.280 0 0 0 0]
    ## d=[0.1283+0.115  0.030  0.020  0.140+0.105  0.0285+0.0285  0.105]
    ## th=[ th1  th2  th3  th4  th5 th6]
    
    # j11=(81*cos(th1 - th2)*cos(th4))/500 - (7*sin(th1))/25 - (81*sin(th1 - th2)*sin(th3 + th4))/1000 - cos(th1 - th2)/50 - (49*sin(th1 - th2)*sin(th3))/200 + (81*sin(th1 - th2)*sin(th3 - th4))/1000
    # j12=cos(th1 - th2)/50 - (81*cos(th1 - th2)*cos(th4))/500 + (49*sin(th1 - th2)*sin(th3))/200 + (81*sin(th1 - th2)*cos(th3)*sin(th4))/500
    # j13=(cos(th1 - th2)*(245*cos(th3) - 162*sin(th3)*sin(th4)))/1000
    # j14=(81*cos(th1)*sin(th2)*sin(th4))/500 - (81*cos(th2)*sin(th1)*sin(th4))/500 + (81*cos(th1)*cos(th2)*cos(th3)*cos(th4))/500 + (81*cos(th3)*cos(th4)*sin(th1)*sin(th2))/500
    # j15=0
    # j16=0

    # j21=0
    # j22=0
    # j23=(49*sin(th3))/200 + (81*cos(th3)*sin(th4))/500
    # j24=(81*cos(th4)*sin(th3))/500
    # j25=0
    # j26=0

    # j31=(7*cos(th1))/25 - sin(th1 - th2)/50 + (81*cos(th1 - th2)*sin(th3 + th4))/1000 + (49*cos(th1 - th2)*sin(th3))/200 + (81*sin(th1 - th2)*cos(th4))/500 - (81*cos(th1 - th2)*sin(th3 - th4))/1000
    # j32=sin(th1 - th2)/50 - (49*cos(th1 - th2)*sin(th3))/200 - (81*sin(th1 - th2)*cos(th4))/500 - (81*cos(th1 - th2)*cos(th3)*sin(th4))/500
    # j33=(sin(th1 - th2)*(245*cos(th3) - 162*sin(th3)*sin(th4)))/1000
    # j34=(81*sin(th1)*sin(th2)*sin(th4))/500 + (81*cos(th1)*cos(th2)*sin(th4))/500 - (81*cos(th1)*cos(th3)*cos(th4)*sin(th2))/500 + (81*cos(th2)*cos(th3)*cos(th4)*sin(th1))/500
    # j35=0
    # j36=0

    # j41=0
    # j42=0
    # j43=-sin(th1 - th2)
    # j44=cos(th1 - th2)*sin(th3)
    # j45=sin(th1 - th2)*cos(th4) + cos(th1 - th2)*cos(th3)*sin(th4)
    # j46=sin(th1 - th2)*cos(th4) + cos(th1 - th2)*cos(th3)*sin(th4)

    # j51=-1
    # j52=1
    # j53=0
    # j54=-cos(th3)
    # j55=sin(th3)*sin(th4)
    # j56=sin(th3)*sin(th4)

    # j61=0
    # j62=0
    # j63=cos(th1 - th2)
    # j64=sin(th1 - th2)*sin(th3)
    # j65=sin(th1 - th2)*cos(th3)*sin(th4) - cos(th1 - th2)*cos(th4)
    # j66=sin(th1 - th2)*cos(th3)*sin(th4) - cos(th1 - th2)*cos(th4)



##---Jacobian with considering same DH given in manual---## 
    ## alpha= [pi/2 pi pi/2 pi/2 pi/2 0]
    ## a=[0.0 0.280 0.0 0.0 0.0 0.0]
    ## d=[0.1283+0.115  0.030  0.020  0.140+0.105  0.0285+0.0285  0.105]
    ## th=[ th1  th2+(pi/2)  th3+(pi/2)  th4+(pi/2)  th5+(pi) th6+(pi/2)]
    
    # j11=cos(th1)/100 - (57*cos(th1)*sin(th4))/1000 - (49*cos(th2)*sin(th1)*sin(th3))/200 + (49*cos(th3)*sin(th1)*sin(th2))/200 + a2*sin(th1)*sin(th2) + (21*cos(th1)*cos(th4)*sin(th5))/200 - (57*cos(th2)*cos(th3)*cos(th4)*sin(th1))/1000 - (21*cos(th2)*cos(th5)*sin(th1)*sin(th3))/200 + (21*cos(th3)*cos(th5)*sin(th1)*sin(th2))/200 - (57*cos(th4)*sin(th1)*sin(th2)*sin(th3))/1000 - (21*cos(th2)*cos(th3)*sin(th1)*sin(th4)*sin(th5))/200 - (21*sin(th1)*sin(th2)*sin(th3)*sin(th4)*sin(th5))/200
    # j12=-(cos(th1)*(245*cos(th2)*cos(th3) + 245*sin(th2)*sin(th3) + 1000*a2*cos(th2) + 105*cos(th5)*sin(th2)*sin(th3) + 105*cos(th2)*cos(th3)*cos(th5) - 57*cos(th2)*cos(th4)*sin(th3) + 57*cos(th3)*cos(th4)*sin(th2) - 105*cos(th2)*sin(th3)*sin(th4)*sin(th5) + 105*cos(th3)*sin(th2)*sin(th4)*sin(th5)))/1000
    # j13=(cos(th1)*(245*cos(th2)*cos(th3) + 245*sin(th2)*sin(th3) + 105*cos(th5)*sin(th2)*sin(th3) + 105*cos(th2)*cos(th3)*cos(th5) - 57*cos(th2)*cos(th4)*sin(th3) + 57*cos(th3)*cos(th4)*sin(th2) - 105*cos(th2)*sin(th3)*sin(th4)*sin(th5) + 105*cos(th3)*sin(th2)*sin(th4)*sin(th5)))/1000
    # j14= (21*cos(th1)*cos(th2)*cos(th3)*cos(th4)*sin(th5))/200 - (21*sin(th1)*sin(th4)*sin(th5))/200 - (57*cos(th1)*cos(th2)*cos(th3)*sin(th4))/1000 - (57*cos(th1)*sin(th2)*sin(th3)*sin(th4))/1000 - (57*cos(th4)*sin(th1))/1000 + (21*cos(th1)*cos(th4)*sin(th2)*sin(th3)*sin(th5))/200
    # j15=(21*cos(th4)*cos(th5)*sin(th1))/200 - (21*cos(th1)*cos(th2)*sin(th3)*sin(th5))/200 + (21*cos(th1)*cos(th3)*sin(th2)*sin(th5))/200 + (21*cos(th1)*cos(th2)*cos(th3)*cos(th5)*sin(th4))/200 + (21*cos(th1)*cos(th5)*sin(th2)*sin(th3)*sin(th4))/200
    # j16=0

    # j21=sin(th1)/100 - (57*sin(th1)*sin(th4))/1000 + (21*cos(th4)*sin(th1)*sin(th5))/200 - a2*cos(th1)*sin(th2) + (49*cos(th1)*cos(th2)*sin(th3))/200 - (49*cos(th1)*cos(th3)*sin(th2))/200 + (57*cos(th1)*cos(th2)*cos(th3)*cos(th4))/1000 + (21*cos(th1)*cos(th2)*cos(th5)*sin(th3))/200 - (21*cos(th1)*cos(th3)*cos(th5)*sin(th2))/200 + (57*cos(th1)*cos(th4)*sin(th2)*sin(th3))/1000 + (21*cos(th1)*cos(th2)*cos(th3)*sin(th4)*sin(th5))/200 + (21*cos(th1)*sin(th2)*sin(th3)*sin(th4)*sin(th5))/200
    # j22=-(sin(th1)*(245*cos(th2)*cos(th3) + 245*sin(th2)*sin(th3) + 1000*a2*cos(th2) + 105*cos(th5)*sin(th2)*sin(th3) + 105*cos(th2)*cos(th3)*cos(th5) - 57*cos(th2)*cos(th4)*sin(th3) + 57*cos(th3)*cos(th4)*sin(th2) - 105*cos(th2)*sin(th3)*sin(th4)*sin(th5) + 105*cos(th3)*sin(th2)*sin(th4)*sin(th5)))/1000
    # j23=(sin(th1)*(245*cos(th2)*cos(th3) + 245*sin(th2)*sin(th3) + 105*cos(th5)*sin(th2)*sin(th3) + 105*cos(th2)*cos(th3)*cos(th5) - 57*cos(th2)*cos(th4)*sin(th3) + 57*cos(th3)*cos(th4)*sin(th2) - 105*cos(th2)*sin(th3)*sin(th4)*sin(th5) + 105*cos(th3)*sin(th2)*sin(th4)*sin(th5)))/1000
    # j24=(57*cos(th1)*cos(th4))/1000 + (21*cos(th1)*sin(th4)*sin(th5))/200 - (57*cos(th2)*cos(th3)*sin(th1)*sin(th4))/1000 - (57*sin(th1)*sin(th2)*sin(th3)*sin(th4))/1000 + (21*cos(th2)*cos(th3)*cos(th4)*sin(th1)*sin(th5))/200 + (21*cos(th4)*sin(th1)*sin(th2)*sin(th3)*sin(th5))/200 
    # j25=(21*cos(th3)*sin(th1)*sin(th2)*sin(th5))/200 - (21*cos(th2)*sin(th1)*sin(th3)*sin(th5))/200 - (21*cos(th1)*cos(th4)*cos(th5))/200 + (21*cos(th2)*cos(th3)*cos(th5)*sin(th1)*sin(th4))/200 + (21*cos(th5)*sin(th1)*sin(th2)*sin(th3)*sin(th4))/200
    # j26=0

    # j31=0
    # j32=(49*cos(th2)*sin(th3))/200 - (49*cos(th3)*sin(th2))/200 - a2*sin(th2) + (57*cos(th4)*sin(th2)*sin(th3))/1000 + (57*cos(th2)*cos(th3)*cos(th4))/1000 + (21*cos(th2)*cos(th5)*sin(th3))/200 - (21*cos(th3)*cos(th5)*sin(th2))/200 + (21*cos(th2)*cos(th3)*sin(th4)*sin(th5))/200 + (21*sin(th2)*sin(th3)*sin(th4)*sin(th5))/200
    # j33=(49*cos(th3)*sin(th2))/200 - (49*cos(th2)*sin(th3))/200 - (57*cos(th4)*sin(th2)*sin(th3))/1000 - (57*cos(th2)*cos(th3)*cos(th4))/1000 - (21*cos(th2)*cos(th5)*sin(th3))/200 + (21*cos(th3)*cos(th5)*sin(th2))/200 - (21*cos(th2)*cos(th3)*sin(th4)*sin(th5))/200 - (21*sin(th2)*sin(th3)*sin(th4)*sin(th5))/200
    # j34=-(3*sin(th2 - th3)*(19*sin(th4) - 35*cos(th4)*sin(th5)))/1000 
    # j35=(21*cos(th3)*cos(th5)*sin(th2)*sin(th4))/200 - (21*cos(th2)*cos(th3)*sin(th5))/200 - (21*cos(th2)*cos(th5)*sin(th3)*sin(th4))/200 - (21*sin(th2)*sin(th3)*sin(th5))/200 
    # j36=0

    # j41=0
    # j42=sin(th1)
    # j43=-sin(th1)
    # j44=-sin(th2 - th3)*cos(th1)
    # j45=cos(th1)*cos(th2)*cos(th3)*cos(th4) - sin(th1)*sin(th4) + cos(th1)*cos(th4)*sin(th2)*sin(th3)
    # j46=cos(th4)*sin(th1)*sin(th5) + cos(th1)*cos(th2)*cos(th5)*sin(th3) - cos(th1)*cos(th3)*cos(th5)*sin(th2) + cos(th1)*cos(th2)*cos(th3)*sin(th4)*sin(th5) + cos(th1)*sin(th2)*sin(th3)*sin(th4)*sin(th5)

    # j51=0
    # j52=-cos(th1)
    # j53=cos(th1)
    # j54=-sin(th2 - th3)*sin(th1)
    # j55=cos(th1)*sin(th4) + cos(th2)*cos(th3)*cos(th4)*sin(th1) + cos(th4)*sin(th1)*sin(th2)*sin(th3)
    # j56=cos(th2)*cos(th5)*sin(th1)*sin(th3) - cos(th1)*cos(th4)*sin(th5) - cos(th3)*cos(th5)*sin(th1)*sin(th2) + cos(th2)*cos(th3)*sin(th1)*sin(th4)*sin(th5) + sin(th1)*sin(th2)*sin(th3)*sin(th4)*sin(th5)
 
    # j61=1
    # j62=0
    # j63=0
    # j64=cos(th2 - th3)
    # j65=sin(th2 - th3)*cos(th4)
    # j66=cos(th5)*sin(th2)*sin(th3) + cos(th2)*cos(th3)*cos(th5) - cos(th2)*sin(th3)*sin(th4)*sin(th5) + cos(th3)*sin(th2)*sin(th4)*sin(th5)
 
### ---Jacobian considering DH given below---##
    ## alpha= [pi/2 pi pi/2 pi/2 pi/2 0]
    ## a=[0 0.280 0 0 0 0]
    ## d=[0.1283+0.115  0.030  0.020  0.140+0.105  0.0285+0.0285  0.105]
    ## th=[ th1  th2  th3  th4  th5 th6]
    
    # j11=cos(th1)/100 + (57*cos(th1)*cos(th4))/1000 - (49*cos(th2)*sin(th1)*sin(th3))/200 + (49*cos(th3)*sin(th1)*sin(th2))/200 - (21*cos(th1)*sin(th4)*sin(th5))/200 - (57*cos(th2 - th3)*sin(th1)*sin(th4))/1000 - (21*sin(th2 - th3)*cos(th5)*sin(th1))/200 - a2*cos(th2)*sin(th1) - (21*cos(th2)*cos(th3)*cos(th4)*sin(th1)*sin(th5))/200 - (21*cos(th4)*sin(th1)*sin(th2)*sin(th3)*sin(th5))/200
    # j12=-(cos(th1)*(245*cos(th2)*cos(th3) + 245*sin(th2)*sin(th3) + 1000*a2*sin(th2) - 57*cos(th2)*sin(th3)*sin(th4) + 57*cos(th3)*sin(th2)*sin(th4) - 105*cos(th5)*sin(th2)*sin(th3) - 105*cos(th2)*cos(th3)*cos(th5) - 105*cos(th2)*cos(th4)*sin(th3)*sin(th5) + 105*cos(th3)*cos(th4)*sin(th2)*sin(th5)))/1000
    # j13=(49*cos(th1)*sin(th2)*sin(th3))/200 - (57*sin(th4)*(cos(th1)*cos(th2)*sin(th3) - cos(th1)*cos(th3)*sin(th2)))/1000 - (21*cos(th5)*(cos(th1)*sin(th2)*sin(th3) + cos(th1)*cos(th2)*cos(th3)))/200 - (21*cos(th4)*sin(th5)*(cos(th1)*cos(th2)*sin(th3) - cos(th1)*cos(th3)*sin(th2)))/200 + (49*cos(th1)*cos(th2)*cos(th3))/200
    # j14=(57*cos(th4)*(cos(th1)*sin(th2)*sin(th3) + cos(th1)*cos(th2)*cos(th3)))/1000 - (21*sin(th5)*(cos(th4)*sin(th1) + sin(th4)*(cos(th1)*sin(th2)*sin(th3) + cos(th1)*cos(th2)*cos(th3))))/200 - (57*sin(th1)*sin(th4))/1000
    # j15=(21*sin(th5)*(cos(th1)*cos(th2)*sin(th3) - cos(th1)*cos(th3)*sin(th2)))/200 - (21*cos(th5)*(sin(th1)*sin(th4) - cos(th4)*(cos(th1)*sin(th2)*sin(th3) + cos(th1)*cos(th2)*cos(th3))))/200
    # j16=0

    # j21=sin(th1)/100 + (57*cos(th4)*sin(th1))/1000 - (21*sin(th1)*sin(th4)*sin(th5))/200 + (57*cos(th2 - th3)*cos(th1)*sin(th4))/1000 + (21*sin(th2 - th3)*cos(th1)*cos(th5))/200 + a2*cos(th1)*cos(th2) + (49*cos(th1)*cos(th2)*sin(th3))/200 - (49*cos(th1)*cos(th3)*sin(th2))/200 + (21*cos(th1)*cos(th2)*cos(th3)*cos(th4)*sin(th5))/200 + (21*cos(th1)*cos(th4)*sin(th2)*sin(th3)*sin(th5))/200
    # j22=-(sin(th1)*(245*cos(th2)*cos(th3) + 245*sin(th2)*sin(th3) + 1000*a2*sin(th2) - 57*cos(th2)*sin(th3)*sin(th4) + 57*cos(th3)*sin(th2)*sin(th4) - 105*cos(th5)*sin(th2)*sin(th3) - 105*cos(th2)*cos(th3)*cos(th5) - 105*cos(th2)*cos(th4)*sin(th3)*sin(th5) + 105*cos(th3)*cos(th4)*sin(th2)*sin(th5)))/1000
    # j23=(49*sin(th1)*sin(th2)*sin(th3))/200 - (57*sin(th4)*(cos(th2)*sin(th1)*sin(th3) - cos(th3)*sin(th1)*sin(th2)))/1000 - (21*cos(th5)*(sin(th1)*sin(th2)*sin(th3) + cos(th2)*cos(th3)*sin(th1)))/200 - (21*cos(th4)*sin(th5)*(cos(th2)*sin(th1)*sin(th3) - cos(th3)*sin(th1)*sin(th2)))/200 + (49*cos(th2)*cos(th3)*sin(th1))/200
    # j24=(57*cos(th1)*sin(th4))/1000 + (21*sin(th5)*(cos(th1)*cos(th4) - sin(th4)*(sin(th1)*sin(th2)*sin(th3) + cos(th2)*cos(th3)*sin(th1))))/200 + (57*cos(th4)*(sin(th1)*sin(th2)*sin(th3) + cos(th2)*cos(th3)*sin(th1)))/1000
    # j25=(21*cos(th5)*(cos(th1)*sin(th4) + cos(th4)*(sin(th1)*sin(th2)*sin(th3) + cos(th2)*cos(th3)*sin(th1))))/200 + (21*sin(th5)*(cos(th2)*sin(th1)*sin(th3) - cos(th3)*sin(th1)*sin(th2)))/200
    # j26=0

    # j31=0
    # j32=(21*cos(th2 - th3)*sin(th4 + th5))/400 - (49*sin(th2 - th3))/200 + (57*cos(th2 - th3)*sin(th4))/1000 + (21*sin(th2 - th3)*cos(th5))/200 - (21*cos(th2 - th3)*sin(th4 - th5))/400 + a2*cos(th2)
    # j33=(49*sin(th2 - th3))/200 - (57*cos(th2 - th3)*sin(th4))/1000 - (21*sin(th2 - th3)*cos(th5))/200 - (21*cos(th2 - th3)*cos(th4)*sin(th5))/200
    # j34=(3*sin(th2 - th3)*(19*cos(th4) - 35*sin(th4)*sin(th5)))/1000
    # j35=(21*cos(th2 - th3)*sin(th5))/200 + (21*sin(th2 - th3)*cos(th4)*cos(th5))/200
    # j36=0

    # j41=0
    # j42=sin(th1)
    # j43=-sin(th1)
    # j44=-sin(th2 - th3)*cos(th1)
    # j45=cos(th4)*sin(th1) + sin(th4)*(cos(th1)*sin(th2)*sin(th3) + cos(th1)*cos(th2)*cos(th3))
    # j46=- sin(th5)*(sin(th1)*sin(th4) - cos(th4)*(cos(th1)*sin(th2)*sin(th3) + cos(th1)*cos(th2)*cos(th3))) - cos(th5)*(cos(th1)*cos(th2)*sin(th3) - cos(th1)*cos(th3)*sin(th2))

    # j51=0
    # j52=-cos(th1)
    # j53=cos(th1)
    # j54=-sin(th2 - th3)*sin(th1)
    # j55=sin(th4)*(sin(th1)*sin(th2)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) - cos(th1)*cos(th4)
    # j56=sin(th5)*(cos(th1)*sin(th4) + cos(th4)*(sin(th1)*sin(th2)*sin(th3) + cos(th2)*cos(th3)*sin(th1))) - cos(th5)*(cos(th2)*sin(th1)*sin(th3) - cos(th3)*sin(th1)*sin(th2))

    # j61=1
    # j62=0
    # j63=0
    # j64=cos(th2 - th3)
    # j65=sin(th2 - th3)*sin(th4)
    # j66=sin(th2 - th3)*cos(th4)*sin(th5) - cos(th2 - th3)*cos(th5)


##---Jacobian with considering same DH given in manual---##
    ## alpha= [pi/2 pi pi/2 pi/2 pi/2 0]
    ## a=[0 0.280 0 0 0 0]
    ## d=[0.1283+0.115  0.030  0.020  0.140+0.105  0.0285+0.0285  0.105+0.130]
    ## th=[ th1  th2  th3  th4  th5 th6]
    
    # j11=cos(th1)/100 + (57*cos(th1)*cos(th4))/1000 - (7*cos(th2)*sin(th1))/25 - (49*cos(th2)*sin(th1)*sin(th3))/200 + (49*cos(th3)*sin(th1)*sin(th2))/200 - (47*cos(th1)*sin(th4)*sin(th5))/200 - (57*cos(th2 - th3)*sin(th1)*sin(th4))/1000 - (47*sin(th2 - th3)*cos(th5)*sin(th1))/200 - (47*cos(th2)*cos(th3)*cos(th4)*sin(th1)*sin(th5))/200 - (47*cos(th4)*sin(th1)*sin(th2)*sin(th3)*sin(th5))/200
    # j12=-(cos(th1)*(280*sin(th2) + 245*cos(th2)*cos(th3) + 245*sin(th2)*sin(th3) - 57*cos(th2)*sin(th3)*sin(th4) + 57*cos(th3)*sin(th2)*sin(th4) - 235*cos(th5)*sin(th2)*sin(th3) - 235*cos(th2)*cos(th3)*cos(th5) - 235*cos(th2)*cos(th4)*sin(th3)*sin(th5) + 235*cos(th3)*cos(th4)*sin(th2)*sin(th5)))/1000
    # j13=(49*cos(th1)*sin(th2)*sin(th3))/200 - (57*sin(th4)*(cos(th1)*cos(th2)*sin(th3) - cos(th1)*cos(th3)*sin(th2)))/1000 - (47*cos(th5)*(cos(th1)*sin(th2)*sin(th3) + cos(th1)*cos(th2)*cos(th3)))/200 - (47*cos(th4)*sin(th5)*(cos(th1)*cos(th2)*sin(th3) - cos(th1)*cos(th3)*sin(th2)))/200 + (49*cos(th1)*cos(th2)*cos(th3))/200
    # j14=(57*cos(th4)*(cos(th1)*sin(th2)*sin(th3) + cos(th1)*cos(th2)*cos(th3)))/1000 - (47*sin(th5)*(cos(th4)*sin(th1) + sin(th4)*(cos(th1)*sin(th2)*sin(th3) + cos(th1)*cos(th2)*cos(th3))))/200 - (57*sin(th1)*sin(th4))/1000
    # j15=(47*sin(th5)*(cos(th1)*cos(th2)*sin(th3) - cos(th1)*cos(th3)*sin(th2)))/200 - (47*cos(th5)*(sin(th1)*sin(th4) - cos(th4)*(cos(th1)*sin(th2)*sin(th3) + cos(th1)*cos(th2)*cos(th3))))/200
    # j16=0

    # j21=sin(th1)/100 + (7*cos(th1)*cos(th2))/25 + (57*cos(th4)*sin(th1))/1000 - (47*sin(th1)*sin(th4)*sin(th5))/200 + (57*cos(th2 - th3)*cos(th1)*sin(th4))/1000 + (47*sin(th2 - th3)*cos(th1)*cos(th5))/200 + (49*cos(th1)*cos(th2)*sin(th3))/200 - (49*cos(th1)*cos(th3)*sin(th2))/200 + (47*cos(th1)*cos(th2)*cos(th3)*cos(th4)*sin(th5))/200 + (47*cos(th1)*cos(th4)*sin(th2)*sin(th3)*sin(th5))/200
    # j22=-(sin(th1)*(280*sin(th2) + 245*cos(th2)*cos(th3) + 245*sin(th2)*sin(th3) - 57*cos(th2)*sin(th3)*sin(th4) + 57*cos(th3)*sin(th2)*sin(th4) - 235*cos(th5)*sin(th2)*sin(th3) - 235*cos(th2)*cos(th3)*cos(th5) - 235*cos(th2)*cos(th4)*sin(th3)*sin(th5) + 235*cos(th3)*cos(th4)*sin(th2)*sin(th5)))/1000
    # j23=(49*sin(th1)*sin(th2)*sin(th3))/200 - (57*sin(th4)*(cos(th2)*sin(th1)*sin(th3) - cos(th3)*sin(th1)*sin(th2)))/1000 - (47*cos(th5)*(sin(th1)*sin(th2)*sin(th3) + cos(th2)*cos(th3)*sin(th1)))/200 - (47*cos(th4)*sin(th5)*(cos(th2)*sin(th1)*sin(th3) - cos(th3)*sin(th1)*sin(th2)))/200 + (49*cos(th2)*cos(th3)*sin(th1))/200
    # j24=(57*cos(th1)*sin(th4))/1000 + (47*sin(th5)*(cos(th1)*cos(th4) - sin(th4)*(sin(th1)*sin(th2)*sin(th3) + cos(th2)*cos(th3)*sin(th1))))/200 + (57*cos(th4)*(sin(th1)*sin(th2)*sin(th3) + cos(th2)*cos(th3)*sin(th1)))/1000
    # j25=(47*cos(th5)*(cos(th1)*sin(th4) + cos(th4)*(sin(th1)*sin(th2)*sin(th3) + cos(th2)*cos(th3)*sin(th1))))/200 + (47*sin(th5)*(cos(th2)*sin(th1)*sin(th3) - cos(th3)*sin(th1)*sin(th2)))/200
    # j26=0

    # j31=0
    # j32=(7*cos(th2))/25 - (49*sin(th2 - th3))/200 + (47*cos(th2 - th3)*sin(th4 + th5))/400 + (57*cos(th2 - th3)*sin(th4))/1000 + (47*sin(th2 - th3)*cos(th5))/200 - (47*cos(th2 - th3)*sin(th4 - th5))/400
    # j33=(49*sin(th2 - th3))/200 - (57*cos(th2 - th3)*sin(th4))/1000 - (47*sin(th2 - th3)*cos(th5))/200 - (47*cos(th2 - th3)*cos(th4)*sin(th5))/200
    # j34=(sin(th2 - th3)*(57*cos(th4) - 235*sin(th4)*sin(th5)))/1000
    # j35=(47*cos(th2 - th3)*sin(th5))/200 + (47*sin(th2 - th3)*cos(th4)*cos(th5))/200
    # j36=0

    # j41=0
    # j42=sin(th1)
    # j43=-sin(th1)
    # j44=-sin(th2 - th3)*cos(th1)
    # j45=cos(th4)*sin(th1) + sin(th4)*(cos(th1)*sin(th2)*sin(th3) + cos(th1)*cos(th2)*cos(th3))
    # j46=- sin(th5)*(sin(th1)*sin(th4) - cos(th4)*(cos(th1)*sin(th2)*sin(th3) + cos(th1)*cos(th2)*cos(th3))) - cos(th5)*(cos(th1)*cos(th2)*sin(th3) - cos(th1)*cos(th3)*sin(th2))

    # j51=0
    # j52=-cos(th1)
    # j53=cos(th1)
    # j54=-sin(th2 - th3)*sin(th1)
    # j55=sin(th4)*(sin(th1)*sin(th2)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) - cos(th1)*cos(th4)
    # j56=sin(th5)*(cos(th1)*sin(th4) + cos(th4)*(sin(th1)*sin(th2)*sin(th3) + cos(th2)*cos(th3)*sin(th1))) - cos(th5)*(cos(th2)*sin(th1)*sin(th3) - cos(th3)*sin(th1)*sin(th2))

    # j61=1
    # j62=0
    # j63=0
    # j64=cos(th2 - th3)
    # j65=sin(th2 - th3)*sin(th4)
    # j66=sin(th2 - th3)*cos(th4)*sin(th5) - cos(th2 - th3)*cos(th5)

##---Jacobian with considering same DH given in manual---## 
    ## alpha= [pi/2 pi pi/2 pi/2 pi/2 0]
    ## a=[0.0 0.280 0.0 0.0 0.0 0.0]
    ## d=[0.1283+0.115  0.030  0.020  0.140+0.105  0.0285+0.0285  0.105+0.130]
    ## th=[ th1  th2+(pi/2)  th3+(pi/2)  th4+(pi/2)  th5+(pi) th6+(pi/2)]

    j11=cos(th1)/100 - (57*cos(th1)*sin(th4))/1000 + (7*sin(th1)*sin(th2))/25 - (49*cos(th2)*sin(th1)*sin(th3))/200 + (49*cos(th3)*sin(th1)*sin(th2))/200 + (47*cos(th1)*cos(th4)*sin(th5))/200 - (57*cos(th2)*cos(th3)*cos(th4)*sin(th1))/1000 - (47*cos(th2)*cos(th5)*sin(th1)*sin(th3))/200 + (47*cos(th3)*cos(th5)*sin(th1)*sin(th2))/200 - (57*cos(th4)*sin(th1)*sin(th2)*sin(th3))/1000 - (47*cos(th2)*cos(th3)*sin(th1)*sin(th4)*sin(th5))/200 - (47*sin(th1)*sin(th2)*sin(th3)*sin(th4)*sin(th5))/200
    j12=-(cos(th1)*(280*cos(th2) + 245*cos(th2)*cos(th3) + 245*sin(th2)*sin(th3) + 235*cos(th5)*sin(th2)*sin(th3) + 235*cos(th2)*cos(th3)*cos(th5) - 57*cos(th2)*cos(th4)*sin(th3) + 57*cos(th3)*cos(th4)*sin(th2) - 235*cos(th2)*sin(th3)*sin(th4)*sin(th5) + 235*cos(th3)*sin(th2)*sin(th4)*sin(th5)))/1000
    j13=(cos(th1)*(245*cos(th2)*cos(th3) + 245*sin(th2)*sin(th3) + 235*cos(th5)*sin(th2)*sin(th3) + 235*cos(th2)*cos(th3)*cos(th5) - 57*cos(th2)*cos(th4)*sin(th3) + 57*cos(th3)*cos(th4)*sin(th2) - 235*cos(th2)*sin(th3)*sin(th4)*sin(th5) + 235*cos(th3)*sin(th2)*sin(th4)*sin(th5)))/1000
    j14=(47*cos(th1)*cos(th2)*cos(th3)*cos(th4)*sin(th5))/200 - (47*sin(th1)*sin(th4)*sin(th5))/200 - (57*cos(th1)*cos(th2)*cos(th3)*sin(th4))/1000 - (57*cos(th1)*sin(th2)*sin(th3)*sin(th4))/1000 - (57*cos(th4)*sin(th1))/1000 + (47*cos(th1)*cos(th4)*sin(th2)*sin(th3)*sin(th5))/200
    j15=(47*cos(th4)*cos(th5)*sin(th1))/200 - (47*cos(th1)*cos(th2)*sin(th3)*sin(th5))/200 + (47*cos(th1)*cos(th3)*sin(th2)*sin(th5))/200 + (47*cos(th1)*cos(th2)*cos(th3)*cos(th5)*sin(th4))/200 + (47*cos(th1)*cos(th5)*sin(th2)*sin(th3)*sin(th4))/200
    j16=0

    j21=sin(th1)/100 - (7*cos(th1)*sin(th2))/25 - (57*sin(th1)*sin(th4))/1000 + (47*cos(th4)*sin(th1)*sin(th5))/200 + (49*cos(th1)*cos(th2)*sin(th3))/200 - (49*cos(th1)*cos(th3)*sin(th2))/200 + (57*cos(th1)*cos(th2)*cos(th3)*cos(th4))/1000 + (47*cos(th1)*cos(th2)*cos(th5)*sin(th3))/200 - (47*cos(th1)*cos(th3)*cos(th5)*sin(th2))/200 + (57*cos(th1)*cos(th4)*sin(th2)*sin(th3))/1000 + (47*cos(th1)*cos(th2)*cos(th3)*sin(th4)*sin(th5))/200 + (47*cos(th1)*sin(th2)*sin(th3)*sin(th4)*sin(th5))/200
    j22=-(sin(th1)*(280*cos(th2) + 245*cos(th2)*cos(th3) + 245*sin(th2)*sin(th3) + 235*cos(th5)*sin(th2)*sin(th3) + 235*cos(th2)*cos(th3)*cos(th5) - 57*cos(th2)*cos(th4)*sin(th3) + 57*cos(th3)*cos(th4)*sin(th2) - 235*cos(th2)*sin(th3)*sin(th4)*sin(th5) + 235*cos(th3)*sin(th2)*sin(th4)*sin(th5)))/1000
    j23=(sin(th1)*(245*cos(th2)*cos(th3) + 245*sin(th2)*sin(th3) + 235*cos(th5)*sin(th2)*sin(th3) + 235*cos(th2)*cos(th3)*cos(th5) - 57*cos(th2)*cos(th4)*sin(th3) + 57*cos(th3)*cos(th4)*sin(th2) - 235*cos(th2)*sin(th3)*sin(th4)*sin(th5) + 235*cos(th3)*sin(th2)*sin(th4)*sin(th5)))/1000
    j24=(57*cos(th1)*cos(th4))/1000 + (47*cos(th1)*sin(th4)*sin(th5))/200 - (57*cos(th2)*cos(th3)*sin(th1)*sin(th4))/1000 - (57*sin(th1)*sin(th2)*sin(th3)*sin(th4))/1000 + (47*cos(th2)*cos(th3)*cos(th4)*sin(th1)*sin(th5))/200 + (47*cos(th4)*sin(th1)*sin(th2)*sin(th3)*sin(th5))/200
    j25=(47*cos(th3)*sin(th1)*sin(th2)*sin(th5))/200 - (47*cos(th2)*sin(th1)*sin(th3)*sin(th5))/200 - (47*cos(th1)*cos(th4)*cos(th5))/200 + (47*cos(th2)*cos(th3)*cos(th5)*sin(th1)*sin(th4))/200 + (47*cos(th5)*sin(th1)*sin(th2)*sin(th3)*sin(th4))/200
    j26=0

    j31=0
    j32=(49*cos(th2)*sin(th3))/200 - (7*sin(th2))/25 - (49*cos(th3)*sin(th2))/200 + (57*cos(th4)*sin(th2)*sin(th3))/1000 + (57*cos(th2)*cos(th3)*cos(th4))/1000 + (47*cos(th2)*cos(th5)*sin(th3))/200 - (47*cos(th3)*cos(th5)*sin(th2))/200 + (47*cos(th2)*cos(th3)*sin(th4)*sin(th5))/200 + (47*sin(th2)*sin(th3)*sin(th4)*sin(th5))/200
    j33=(49*cos(th3)*sin(th2))/200 - (49*cos(th2)*sin(th3))/200 - (57*cos(th4)*sin(th2)*sin(th3))/1000 - (57*cos(th2)*cos(th3)*cos(th4))/1000 - (47*cos(th2)*cos(th5)*sin(th3))/200 + (47*cos(th3)*cos(th5)*sin(th2))/200 - (47*cos(th2)*cos(th3)*sin(th4)*sin(th5))/200 - (47*sin(th2)*sin(th3)*sin(th4)*sin(th5))/200
    j34=-(sin(th2 - th3)*(57*sin(th4) - 235*cos(th4)*sin(th5)))/1000
    j35=(47*cos(th3)*cos(th5)*sin(th2)*sin(th4))/200 - (47*cos(th2)*cos(th3)*sin(th5))/200 - (47*cos(th2)*cos(th5)*sin(th3)*sin(th4))/200 - (47*sin(th2)*sin(th3)*sin(th5))/200
    j36=0

    j41=0
    j42=sin(th1)
    j43=-sin(th1)
    j44=-sin(th2 - th3)*cos(th1)
    j45=cos(th1)*cos(th2)*cos(th3)*cos(th4) - sin(th1)*sin(th4) + cos(th1)*cos(th4)*sin(th2)*sin(th3)
    j46=cos(th4)*sin(th1)*sin(th5) + cos(th1)*cos(th2)*cos(th5)*sin(th3) - cos(th1)*cos(th3)*cos(th5)*sin(th2) + cos(th1)*cos(th2)*cos(th3)*sin(th4)*sin(th5) + cos(th1)*sin(th2)*sin(th3)*sin(th4)*sin(th5)

    j51=0
    j52=-cos(th1)
    j53=cos(th1)
    j54=-sin(th2 - th3)*sin(th1)
    j55=cos(th1)*sin(th4) + cos(th2)*cos(th3)*cos(th4)*sin(th1) + cos(th4)*sin(th1)*sin(th2)*sin(th3)
    j56=cos(th2)*cos(th5)*sin(th1)*sin(th3) - cos(th1)*cos(th4)*sin(th5) - cos(th3)*cos(th5)*sin(th1)*sin(th2) + cos(th2)*cos(th3)*sin(th1)*sin(th4)*sin(th5) + sin(th1)*sin(th2)*sin(th3)*sin(th4)*sin(th5)

    j61=1
    j62=0
    j63=0
    j64=cos(th2 - th3)
    j65=sin(th2 - th3)*cos(th4)
    j66=cos(th5)*sin(th2)*sin(th3) + cos(th2)*cos(th3)*cos(th5) - cos(th2)*sin(th3)*sin(th4)*sin(th5) + cos(th3)*sin(th2)*sin(th4)*sin(th5)


    jacobian = np.array([[j11,j12,j13,j14,j15,j16],[j21,j22,j23,j24,j25,j26],[j31,j32,j33,j34,j35,j36],[j41,j42,j43,j44,j45,j46],[j51,j52,j53,j54,j55,j56],[j61,j62,j63,j64,j65,j66]])	
    # jacobian = jacobian.round(3)
    def trunc(values, decs=0):
        return np.trunc(values*10**decs)/(10**decs)
    jacobian = trunc(jacobian, decs=3)
    return jacobian

