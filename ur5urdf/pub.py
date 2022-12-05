#!/usr/bin/env python3

#-------------------------------Importation of required libraries-------------------------------------
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sympy import *
import matplotlib.pyplot as plt
import numpy as np
import math


def pickplace():

    #Initializing Node
    rospy.init_node('publish_node', anonymous=True) # defining the ros node - publish node
    turning = rospy.Publisher('turn', Float64MultiArray, queue_size=10) 
    rate = rospy.Rate(10) # 10hz # fequency at which the publishing occurs
    rospy.loginfo("Analysing the Robot!!!")  # to print on the terminal



    #------------------------------Declaration of variables and symbols-----------------------------------

    theta_1, theta_2, theta_3, theta_4, theta_5, theta_6  = symbols('th1, th2, th3, th4, th5, th6', real=True)
    theta = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]




    #--------------------------Giving the values of d, a, alpha instead of symbols------------------------------------
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
    a[2] = 0.39225
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


    #-----------------------------------Upper half of jacobian------------------------------
    p1 = diff(xp, theta[0])
    p2 = diff(xp, theta[1])
    p3 = diff(xp, theta[2])
    p4 = diff(xp, theta[3])
    p5 = diff(xp, theta[4])
    p6 = diff(xp, theta[5])

    #--------------------------------------Jacobian Matrix----------------------------------
    jac = Matrix([[p1[0], p2[0], p3[0], p4[0], p5[0], p6[0]], [p1[1], p2[1], p3[1], p4[1], p5[1], p6[1]], [p1[2], p2[2], p3[2], p4[2], p5[2], p6[2]], 
                    [z1[0], z2[0], z3[0], z4[0], z5[0], z6[0]], [z1[1], z2[1], z3[1], z4[1], z5[1], z6[1]], [z1[2], z2[2], z3[2], z4[2], z5[2], z6[2]]])
    print("\n\n==========================Jacobian=========================\n\n")
    pprint(jac)

    #--------------------------Iteration of Jacobian and Velocity Matrix--------------------
    theta_inst = [0.00001, pi/10 + 0.00001, (-(pi/2) - (pi/6) - 0.00001), pi/3 + 0.00001, (-pi/2 + 0.00001), 4*pi + 0.00001]


#============================================================ TASK 1 =========================================================
# ------------------------------------------------------ To the Pick position -----------------------------------------------------
    for t in range(10):
        twist = Float64MultiArray()
        twist.data = theta_inst
        turning.publish(twist)
        rate.sleep()


# ------------------------------------------------------ To put partial_position -----------------------------------------------------
    for t in range(20):
        twist = Float64MultiArray()
        q_dot = [-pi/20, (pi/10)/20 , (pi/2 + pi/6 + pi/5)/20, -(pi/3)/20, 0, 0]

        for i in range(len(q_dot)):
            theta_inst[i] += q_dot[i]

        twist.data = theta_inst
        turning.publish(twist)
        rate.sleep()


# ------------------------------------------------------ To put final_position -----------------------------------------------------
    for t in range(20):
        twist = Float64MultiArray()
        q_dot = [0, (pi/10)/20, -(pi/20)/20, -(pi/2)/30, 0, 0]

        for i in range(len(q_dot)):
            theta_inst[i] += q_dot[i]

        twist.data = theta_inst
        turning.publish(twist)
        rate.sleep()




# #============================================================ TASK 2 =========================================================
    # theta_inst = [3*pi/8 + 0.00001, pi/10 + 0.00001, (-(pi/2) - (pi/6) - 0.00001), pi/3 + 0.00001, (-pi/2 + 0.00001), 0.00001]

# # ------------------------------------------------------ To the Pick position -----------------------------------------------------
#     for t in range(10):
#         twist = Float64MultiArray()
#         twist.data = theta_inst
#         turning.publish(twist)
#         rate.sleep()


# # ------------------------------------------------------ To put partial_position -----------------------------------------------------
#     for t in range(20):
#         twist = Float64MultiArray()
#         q_dot = [pi/50, (pi/10)/20 , (pi/2 + pi/6)/20, -(pi/3)/20, 0, 0]

#         for i in range(len(q_dot)):
#             theta_inst[i] += q_dot[i]

#         twist.data = theta_inst
#         turning.publish(twist)
#         rate.sleep()


# # ------------------------------------------------------ To put final_position -----------------------------------------------------
#     for t in range(20):
#         twist = Float64MultiArray()
#         q_dot = [pi/50, -(pi/10)/20 , -(pi/2 + pi/6)/20, (pi/3)/20, 0, 0]

#         for i in range(len(q_dot)):
#             theta_inst[i] += q_dot[i]

#         twist.data = theta_inst
#         turning.publish(twist)
#         rate.sleep()



if __name__ == '__main__':
    try:
        pickplace()
    except rospy.ROSInterruptException: 
        pass
