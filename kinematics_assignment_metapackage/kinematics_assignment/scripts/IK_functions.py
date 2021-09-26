#! /usr/bin/env python3
from math import atan2, acos, sin, cos, asin, pi
import numpy as np
"""
    # Chaitanya Devidas Gore
    # gore@kth.se
"""

l0=0.07
l1=0.3
l2=0.35

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    """
    Fill in your IK solution here and return the three joint values in q
    """
    q[2]= z
    q[1]= acos(((x-0.07)*(x-0.07)+y*y-l1*l1-l2*l2)/(2*l1*l2))

    #q[1]= pi-acos((l1*l1)+(l2*l2)+((x-0.07)*(x-0.07))-(y*y)/(2*l1*l2))
    q[0]= atan2(y,(x-0.07))- atan2((l2*sin(q[1])), (l1+l2*cos(q[1])))




    return q


def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    
    #Fill in your IK solution here and return the seven joint values in q
    



    E = np.ones(6)


    euler= kuka_euler(np.array([R])[0]) # to find eulers angles


    cartecian= np.hstack((np.array([x, y,z]), euler)) # merging of the cartecian points with euler angles


    #update of q value
    for i in range(5):
        x_t,J = kuka_FK(q)
        E= x_t- cartecian
        J= np.linalg.pinv(J)
        E_theta=np.matmul(J, E)

        q= q- E_theta

    #other calculations

    """
    #pbe[x, y, z]=0 # need to write correctly
    #q[0]= atan2(pbe.y , pbe.x)
    q[1]= acos((pbe.z-K)/L)

    [m, n, p, 1]= 0 # need to write correctly
    q[2]= atan2(n,m)
    q[3]= acos((p-L)/M)

    [a,b,c,1]=0 # need to calculate correctly
    q[4]= atan2(b,a)
    q[5]= acos((c-M)/N)

    [r,s,t,1]= 0 # need to calculate correctly

    q[6]=atan2(r,s)
    """


    return q

def kuka_FK(q):
    K=0.311
    L=0.4
    M=0.39
    N=0.078

    #all alpha angle from table

    A= [pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2, 0]

    D= [0,0,L,0,M,0,0]
    p=[]
    z=[]

    T=np.eye(4)
    T[2,3]= K

    p.append(T[0:3,3])
    z.append(T[0:3,2])

    TN=np.eye(4)
    TN[2,3]= N

    #7 value of length of q
    for i in range(len(q)):
        t_mat= np.array( [ [cos(q[i]), -sin(q[i])*cos(A[i]), sin(q[i])*sin(A[i]), 0], 
        [sin(q[i]), cos(q[i])*cos(A[i]), -cos(q[i])*sin(A[i]),0],
        [0, sin(A[i]), cos(A[i]), D[i]],
        [0,0,0,1]])

        T= np.dot(T ,t_mat)
        p.append(T[0:3,3])
        z.append(T[0:3,2])
    T= np.dot(T ,TN)
    p.append(T[0:3,3])
    z.append(T[0:3,2])

    TF=T

    euler_t= kuka_euler(TF[0:3, 0:3])
    x_t=np.hstack((np.array([TF[0][3], TF[1][3], TF[2][3]]), euler_t))

    J=[]

    for i in range(len(q)):
        J_t= np.cross(z[i], (p[-1]- p[i]))
        J.append(np.hstack((J_t, z[i])))
    JA= np.array(J)
    return x_t, JA.T



def close(x,y,rtol=1.e-8, atol=1.e-13):
    return abs(x-y) <= atol*abs(y)


def kuka_euler(R):

    phi=0.0

    if close(R[2,0], -1):
        theta= pi/2
        sci=atan2(R[0,1], R[0,2])
    elif close(R[2,0], 1):
        theta= -pi/2
        sci= atan2(-R[0,1], -R[0,2])

    else:
        theta=-asin(R[2,0])
        sci=atan2(R[2,1]/cos(theta), R[2,2]/cos(theta))
        phi= atan2(R[1,0]/cos(theta), R[0,0]/cos(theta))

    return np.array([sci, theta, phi])

