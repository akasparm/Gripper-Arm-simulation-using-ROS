#!/usr/bin/env python3

#-------------------------------Importation of required libraries-------------------------------------

from sympy import *
import matplotlib.pyplot as plt
import numpy as np
import math


#------------------------------Declaration of variables and symbols-----------------------------------
theta_1, theta_2, theta_3, theta_4, theta_5, theta_6  = symbols('th1, th2, th3, th4, th5, th6', real=True)
theta = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]

#--------------------------Giving the values of d, a, alpha as DH params------------------------------------
d = [0]*7
d[0] = 0.089159
d[1] = 0
d[2] = 0
d[3] = 0.10915
d[4] = 0.09465
d[5] = 0.0823

a = [0]*7
a[0] = 0
a[1] = -0.425
a[2] = -0.39225
a[3] = 0
a[4] = 0
a[5] = 0

alpha = [0]*7
alpha[0] = pi/2
alpha[1] = 0
alpha[2] = 0
alpha[3] = pi/2
alpha[4] = -pi/2
alpha[5] = 0

#-----------------------------Calculations of Homogeneous Transformation Matrices---------------------------------
T_calc = []
for i in range(6):
    x = Matrix([[cos(theta[i]), -sin(theta[i])*cos(alpha[i]), sin(theta[i])*sin(alpha[i]), a[i]*cos(theta[i])],
        [sin(theta[i]), cos(theta[i])*cos(alpha[i]), -cos(theta[i])*sin(alpha[i]), a[i]*sin(theta[i])],
        [0, sin(alpha[i]), cos(alpha[i]), d[i]],
        [0, 0, 0, 1]
        ])
    T_calc.append(x)

final_trans = T_calc[0]*T_calc[1]*T_calc[2]*T_calc[3]*T_calc[4]*T_calc[5]
final_trans = final_trans.evalf()

#---------------------------------T01--------------------------------------
final_trans = T_calc[0]
final_trans = final_trans.evalf()
z1 = final_trans[0:3,2]

#---------------------------------T02--------------------------------------
final_trans = T_calc[0]*T_calc[1]
final_trans = final_trans.evalf()
z2 = final_trans[0:3,2]

#---------------------------------T03--------------------------------------
final_trans = T_calc[0]*T_calc[1]*T_calc[2]
final_trans = final_trans.evalf()
z3 = final_trans[0:3,2]

#---------------------------------T04--------------------------------------
final_trans = T_calc[0]*T_calc[1]*T_calc[2]*T_calc[3]
final_trans = final_trans.evalf()
z4 = final_trans[0:3,2]

#---------------------------------T05--------------------------------------
final_trans = T_calc[0]*T_calc[1]*T_calc[2]*T_calc[3]*T_calc[4]
final_trans = final_trans.evalf()
z5 = final_trans[0:3,2]

#---------------------------------T06--------------------------------------
final_trans = T_calc[0]*T_calc[1]*T_calc[2]*T_calc[3]*T_calc[4]*T_calc[5]
final_trans = final_trans.evalf()
z6 = final_trans[0:3,2]
xp = final_trans[0:3, 3]

a = (T_calc[0]*T_calc[1]*T_calc[2]*T_calc[3]*T_calc[4]*T_calc[5])
a = a.subs(theta_1, 0).subs(theta_2, 0).subs(theta_3, 0).subs(theta_4, 0).subs(theta_5, 0).subs(theta_6, 0)
pprint(a.evalf())