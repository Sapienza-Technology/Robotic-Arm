# -*- coding: utf-8 -*-
"""sasa_functions.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/13oSgkynQoILIi8eZFAqq_eJ6I8NNFxnK
"""

#Definizione costanti e task
from array import array
import math
import string
from tabnanny import check
from numpy.core.fromnumeric import transpose
import numpy as np 
from math import sin, cos, pow, atan2, sqrt, pi 
import numpy as np
import math as m
from spatialmath import SO3

from trajectory import Trajectory

## Useful rotations ####################################################
def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, m.cos(theta),-m.sin(theta) ],
                   [ 0, m.sin(theta), m.cos(theta)]])

def Ry(theta):
  return np.matrix([[ m.cos(theta), 0, m.sin(theta)],
                   [ 0           , 1, 0            ],
                   [-m.sin(theta), 0, m.cos(theta)]])

def Rz(theta):
  return np.matrix([[ m.cos(theta), -m.sin(theta), 0],
                   [ m.sin(theta), m.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])

#####################################################################
## robot constants ###################################################
a0=0.0100
d0=0.0655
l1=0.2950
l2=0.3610
pirul = -0.145
maintenance = -0.121
#####################################################################

#Funzioni principali

def inv_kin_bis(coord:np.array,elbow:bool)->np.array:
    all_q0=np.array([atan2(coord[1],coord[0]),atan2(-coord[1],-coord[0])])
    #q0=atan2(coord[1],coord[0])
    c2=(pow(m.sqrt(pow(coord[0],2)+pow(coord[1],2))-a0,2)+pow(coord[2]-d0,2)-pow(l1,2)-pow(l2,2))/(2*l1*l2)

    if abs(c2) > 1:
      print("out of workspace")
      new_pos = out_of_range_exeption(coord)
      return inv_kin_bis(new_pos, elbow)
    
    c2_minus=(pow(m.sqrt(pow(coord[0],2)+pow(coord[1],2))+a0,2)+pow(coord[2]-d0,2)-pow(l1,2)-pow(l2,2))/(2*l1*l2) #c2 with minus sign
    s2=sqrt(1-pow(c2,2))
    s2_minus=-sqrt(1-pow(c2,2))                              # s2 with minus sign
    all_q2=np.array([])                                      # array containing all possible solutions 
    allA=[]#np.array([[],[]])                                # array containign all possible A
    allso=[]                                                 # array of all so
    all_c2=np.array([c2,c2_minus])                           # array with possible c2
    all_s2=np.array([s2,s2_minus])                           # array with possible s2
    all_q1=np.array([])                                      
    for q0 in all_q0:                                    
      vec=np.matrix([[cos(q0)*coord[0]+sin(q0)*coord[1]-a0], [coord[2]-d0]])
      for i in all_c2:                                         #
        for j in all_s2:                                       #
          q2=atan2(j,i)                                        # q2
          A=np.matrix([[l1+l2*i, -l2*j], [l2*j, l1+l2*i]])     # A
          so=np.matmul(np.linalg.inv(A),vec)                   # so 
          c1=so[0,0]                                           # 
          s1=so[1,0]                                           # 
          q1=atan2(s1,c1)                                      #   
          allso.append(so)                                     # append so array                
          allA.append(A)                                       # append A array
          all_q2=np.append(all_q2, q2)                         # append q2 array
          all_q1=np.append(all_q1, q1)                         # append q1 array                       
    all_q0=np.array([all_q0[0],all_q0[0],all_q0[0],all_q0[0],all_q0[1],all_q0[1],all_q0[1],all_q0[1]])
    all_q=np.array([all_q0, all_q1, all_q2])
    q=best_qp(all_q, elbow, coord)
    return q
    # ora abbiamo tutti i possibili risultati di q2 A so q1 forse è necessario solo q1 e q2 vediamo come usarle

def out_of_range_exeption(coord:np.array)->np.array:
  y = sqrt(pow(l1+l2+0.05,2)/(pow(coord[0],2)/pow(coord[1],2) + pow(coord[2],2)/pow(coord[1],2) + 1))
  if coord[1] < 0:
    y = -y
  x = y*coord[0]/coord[1]
  z = y*coord[2]/coord[1]
  return np.array([x,y,z])


def inv_kin_total_bis(position:np.array, Rtarg:np.matrix, phase, elbow:bool, prec_qfin:np.array)->np.array:
    Ptarg = np.column_stack([Rtarg, position.transpose()])
    Ptarg = np.row_stack([Ptarg, np.array([0,0,0,1])])
    if phase:
      dist_ee = np.array([0,0,pirul,1])
    else:
      dist_ee = np.array([0,0,maintenance,1])

    print("Rf3 e-e: ",dist_ee)
    pint = np.dot(Ptarg, dist_ee)
    pint = np.array([pint[0,0], pint[0,1], pint[0,2]])
    q=inv_kin_bis(pint,elbow)
    #print("dir kin intermedia:")
    #print(dir_kin(q))
    #print("dir kin voluta:")
    #print(pint)
    #print("\n")
    q1=q[0]
    q2=q[1]
    q3=q[2]
    Pint = np.matrix(np.array([[ sin(q2 + q3)*cos(q1), -sin(q1), cos(q2 + q3)*cos(q1),  (cos(q1)*(361*cos(q2 + q3) + 295*cos(q2) + 10))/1000],
                               [ sin(q2 + q3)*sin(q1),  cos(q1), cos(q2 + q3)*sin(q1),  (sin(q1)*(361*cos(q2 + q3) + 295*cos(q2) + 10))/1000],
                               [        -cos(q2 + q3),        0,         sin(q2 + q3), (361*sin(q2 + q3))/1000 + (59*sin(q2))/200 + 131/2000],
                               [                    0,        0,                    0,                                                     1]]))
    Rint = Pint[0:3, 0:3]
    Rsol = np.matmul(Rint.transpose(), Rtarg)
    c5 = Rsol[2,2]
    s5 = sqrt(1-pow(c5,2))
    q4 = atan2(-Rsol[1,2], -Rsol[0,2])
    q4_minus = atan2(Rsol[1,2], Rsol[0,2])
    q5 = atan2(s5,c5)
    q5_minus = atan2(-s5,c5)
    q6 = atan2(-Rsol[2,1], Rsol[2,0])
    q6_minus = atan2(Rsol[2,1], -Rsol[2,0])
    q_op = np.array([q4,q5,q6])
    q_op_minus = np.array([q4_minus, q5_minus, q6_minus])

    #print("Ultime q: ")
    #print(q_op)
    #print(q_op_minus)
    #print("\n")

    if np.linalg.norm(q_op-prec_qfin) > np.linalg.norm(q_op_minus-prec_qfin):
      q4 = q4_minus
      q5 = q5_minus
      q6 = q6_minus

    q = np.array([q1,q2,q3,q4,q5,q6])
    return q

def dir_kin(q : np.array) -> np.array :    
    x=(a0+l1*cos(q[1])+l2*cos(q[1]+q[2]))*cos(q[0])
    y=(a0+l1*cos(q[1])+l2*cos(q[1]+q[2]))*sin(q[0])
    z=d0+l1*sin(q[1])+l2*sin(q[1]+q[2])
    coord=np.array([x,y,z])
    return coord

def dir_kin_total(q:np.array, phase) -> np.array:
  q1 = q[0]
  q2 = q[1]
  q3 = q[2]
  q4 = q[3]
  q5 = q[4]
  q6 = q[5]

  if phase:
    x = cos(q1)/100 + (59*cos(q1)*cos(q2))/200 + (29*sin(q1)*sin(q4)*sin(q5))/200 + (361*cos(q1)*cos(q2)*cos(q3))/1000 - (361*cos(q1)*sin(q2)*sin(q3))/1000 + (29*cos(q1)*cos(q2)*cos(q3)*cos(q5))/200 - (29*cos(q1)*cos(q5)*sin(q2)*sin(q3))/200 - (29*cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q5))/200 - (29*cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5))/200
    y = sin(q1)/100 + (59*cos(q2)*sin(q1))/200 - (361*sin(q1)*sin(q2)*sin(q3))/1000 + (361*cos(q2)*cos(q3)*sin(q1))/1000 - (29*cos(q1)*sin(q4)*sin(q5))/200 + (29*cos(q2)*cos(q3)*cos(q5)*sin(q1))/200 - (29*cos(q5)*sin(q1)*sin(q2)*sin(q3))/200 - (29*cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q5))/200 - (29*cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5))/200
    z = (59*sin(q2))/200 + (361*cos(q2)*sin(q3))/1000 + (361*cos(q3)*sin(q2))/1000 + (29*cos(q2)*cos(q5)*sin(q3))/200 + (29*cos(q3)*cos(q5)*sin(q2))/200 + (29*cos(q2)*cos(q3)*cos(q4)*sin(q5))/200 - (29*cos(q4)*sin(q2)*sin(q3)*sin(q5))/200 + 131/2000

    #x = a1*cos(q1) + l1*cos(q1)*cos(q2) + l2*cos(alpha3)*sin(q1) - pirul*cos(alpha3)*cos(q5)*sin(q1) - l2*cos(q1)*cos(q2)*cos(q3)*sin(alpha3) + l2*cos(q1)*sin(alpha3)*sin(q2)*sin(q3) + pirul*sin(alpha3)*sin(q1)*sin(q4)*sin(q5) + pirul*cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q5) + pirul*cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5) - pirul*cos(q1)*cos(q5)*sin(alpha3)*sin(q2)*sin(q3) + pirul*cos(q1)*cos(q2)*cos(q3)*cos(q5)*sin(alpha3) + pirul*cos(alpha3)*cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q5) - pirul*cos(alpha3)*cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5)
    #y = a1*sin(q1) - l2*cos(alpha3)*cos(q1) + l1*cos(q2)*sin(q1) + pirul*cos(alpha3)*cos(q1)*cos(q5) - l2*cos(q2)*cos(q3)*sin(alpha3)*sin(q1) - pirul*cos(q1)*sin(alpha3)*sin(q4)*sin(q5) + l2*sin(alpha3)*sin(q1)*sin(q2)*sin(q3) + pirul*cos(q2)*cos(q3)*cos(q5)*sin(alpha3)*sin(q1) + pirul*cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q5) + pirul*cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5) - pirul*cos(q5)*sin(alpha3)*sin(q1)*sin(q2)*sin(q3) - pirul*cos(alpha3)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + pirul*cos(alpha3)*cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q5)
    #z = d1 + l1*sin(q2) - l2*cos(q2)*sin(alpha3)*sin(q3) - l2*cos(q3)*sin(alpha3)*sin(q2) - pirul*cos(q2)*cos(q3)*cos(q4)*sin(q5) + pirul*cos(q2)*cos(q5)*sin(alpha3)*sin(q3) + pirul*cos(q3)*cos(q5)*sin(alpha3)*sin(q2) + pirul*cos(q4)*sin(q2)*sin(q3)*sin(q5) + pirul*cos(alpha3)*cos(q2)*sin(q3)*sin(q4)*sin(q5) + pirul*cos(alpha3)*cos(q3)*sin(q2)*sin(q4)*sin(q5)


  else:
    x = cos(q1)/100 + (59*cos(q1)*cos(q2))/200 + (121*sin(q1)*sin(q4)*sin(q5))/1000 + (361*cos(q1)*cos(q2)*cos(q3))/1000 - (361*cos(q1)*sin(q2)*sin(q3))/1000 + (121*cos(q1)*cos(q2)*cos(q3)*cos(q5))/1000 - (121*cos(q1)*cos(q5)*sin(q2)*sin(q3))/1000 - (121*cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q5))/1000 - (121*cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5))/1000
    y = sin(q1)/100 + (59*cos(q2)*sin(q1))/200 - (361*sin(q1)*sin(q2)*sin(q3))/1000 + (361*cos(q2)*cos(q3)*sin(q1))/1000 - (121*cos(q1)*sin(q4)*sin(q5))/1000 + (121*cos(q2)*cos(q3)*cos(q5)*sin(q1))/1000 - (121*cos(q5)*sin(q1)*sin(q2)*sin(q3))/1000 - (121*cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q5))/1000 - (121*cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5))/1000
    z = (59*sin(q2))/200 + (361*cos(q2)*sin(q3))/1000 + (361*cos(q3)*sin(q2))/1000 + (121*cos(q2)*cos(q5)*sin(q3))/1000 + (121*cos(q3)*cos(q5)*sin(q2))/1000 + (121*cos(q2)*cos(q3)*cos(q4)*sin(q5))/1000 - (121*cos(q4)*sin(q2)*sin(q3)*sin(q5))/1000 + 131/2000

  coord=np.array([x,y,z]) 
  return coord

def dir_kin_total_with_rot(q:np.array, phase) -> np.array:
  q1 = q[0]
  q2 = q[1]
  q3 = q[2]
  q4 = q[3]
  q5 = q[4]
  q6 = q[5]
  
  rot = np.matrix([[ cos(q6)*(cos(q2 + q3)*cos(q1)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q2)*cos(q4)*cos(q5)*sin(q3) + cos(q1)*cos(q3)*cos(q4)*cos(q5)*sin(q2)) - sin(q6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*sin(q3)*sin(q4) + cos(q1)*cos(q3)*sin(q2)*sin(q4)), - cos(q6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*sin(q3)*sin(q4) + cos(q1)*cos(q3)*sin(q2)*sin(q4)) - sin(q6)*(cos(q2 + q3)*cos(q1)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q2)*cos(q4)*cos(q5)*sin(q3) + cos(q1)*cos(q3)*cos(q4)*cos(q5)*sin(q2)), cos(q2 + q3)*cos(q1)*cos(q5) - sin(q5)*(cos(q1)*cos(q2)*cos(q4)*sin(q3) - sin(q1)*sin(q4) + cos(q1)*cos(q3)*cos(q4)*sin(q2))],
                   [ cos(q6)*(cos(q2 + q3)*sin(q1)*sin(q5) + cos(q1)*cos(q5)*sin(q4) + cos(q2)*cos(q4)*cos(q5)*sin(q1)*sin(q3) + cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q2)) - sin(q6)*(cos(q2)*sin(q1)*sin(q3)*sin(q4) - cos(q1)*cos(q4) + cos(q3)*sin(q1)*sin(q2)*sin(q4)), - cos(q6)*(cos(q2)*sin(q1)*sin(q3)*sin(q4) - cos(q1)*cos(q4) + cos(q3)*sin(q1)*sin(q2)*sin(q4)) - sin(q6)*(cos(q2 + q3)*sin(q1)*sin(q5) + cos(q1)*cos(q5)*sin(q4) + cos(q2)*cos(q4)*cos(q5)*sin(q1)*sin(q3) + cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q2)), cos(q2 + q3)*cos(q5)*sin(q1) - sin(q5)*(cos(q1)*sin(q4) + cos(q2)*cos(q4)*sin(q1)*sin(q3) + cos(q3)*cos(q4)*sin(q1)*sin(q2))],
                   [                                                                                                                                                         cos(q6)*(sin(q2 + q3)*sin(q5) - cos(q2 + q3)*cos(q4)*cos(q5)) + cos(q2 + q3)*sin(q4)*sin(q6),                                                                                                                                                           cos(q2 + q3)*cos(q6)*sin(q4) - sin(q6)*(sin(q2 + q3)*sin(q5) - cos(q2 + q3)*cos(q4)*cos(q5)),                                                                          sin(q2 + q3)*cos(q5) + cos(q2 + q3)*cos(q4)*sin(q5)]])

  if phase:
    x = cos(q1)/100 + (59*cos(q1)*cos(q2))/200 + (29*sin(q1)*sin(q4)*sin(q5))/200 + (361*cos(q1)*cos(q2)*cos(q3))/1000 - (361*cos(q1)*sin(q2)*sin(q3))/1000 + (29*cos(q1)*cos(q2)*cos(q3)*cos(q5))/200 - (29*cos(q1)*cos(q5)*sin(q2)*sin(q3))/200 - (29*cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q5))/200 - (29*cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5))/200
    y = sin(q1)/100 + (59*cos(q2)*sin(q1))/200 - (361*sin(q1)*sin(q2)*sin(q3))/1000 + (361*cos(q2)*cos(q3)*sin(q1))/1000 - (29*cos(q1)*sin(q4)*sin(q5))/200 + (29*cos(q2)*cos(q3)*cos(q5)*sin(q1))/200 - (29*cos(q5)*sin(q1)*sin(q2)*sin(q3))/200 - (29*cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q5))/200 - (29*cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5))/200
    z = (59*sin(q2))/200 + (361*cos(q2)*sin(q3))/1000 + (361*cos(q3)*sin(q2))/1000 + (29*cos(q2)*cos(q5)*sin(q3))/200 + (29*cos(q3)*cos(q5)*sin(q2))/200 + (29*cos(q2)*cos(q3)*cos(q4)*sin(q5))/200 - (29*cos(q4)*sin(q2)*sin(q3)*sin(q5))/200 + 131/2000

  else:
    x = cos(q1)/100 + (59*cos(q1)*cos(q2))/200 + (121*sin(q1)*sin(q4)*sin(q5))/1000 + (361*cos(q1)*cos(q2)*cos(q3))/1000 - (361*cos(q1)*sin(q2)*sin(q3))/1000 + (121*cos(q1)*cos(q2)*cos(q3)*cos(q5))/1000 - (121*cos(q1)*cos(q5)*sin(q2)*sin(q3))/1000 - (121*cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q5))/1000 - (121*cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5))/1000
    y = sin(q1)/100 + (59*cos(q2)*sin(q1))/200 - (361*sin(q1)*sin(q2)*sin(q3))/1000 + (361*cos(q2)*cos(q3)*sin(q1))/1000 - (121*cos(q1)*sin(q4)*sin(q5))/1000 + (121*cos(q2)*cos(q3)*cos(q5)*sin(q1))/1000 - (121*cos(q5)*sin(q1)*sin(q2)*sin(q3))/1000 - (121*cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q5))/1000 - (121*cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5))/1000
    z = (59*sin(q2))/200 + (361*cos(q2)*sin(q3))/1000 + (361*cos(q3)*sin(q2))/1000 + (121*cos(q2)*cos(q5)*sin(q3))/1000 + (121*cos(q3)*cos(q5)*sin(q2))/1000 + (121*cos(q2)*cos(q3)*cos(q4)*sin(q5))/1000 - (121*cos(q4)*sin(q2)*sin(q3)*sin(q5))/1000 + 131/2000

  coord=np.array([x,y,z]) 
  return rot, coord
 

#funzioni ausiliarie
def range_check(q:np.array):
  res = True  #limiti
  bool_vec = np.ones(3)
  if q[0]>5*pi/4:      
    res = False
    bool_vec[0] = 0
  if q[1]<-0.2618 or q[1]>2.2689:
    res = False
    bool_vec[1] = 0
  if q[2]<-1.9199 or q[2]>1.9199:
    res = False
    bool_vec[2] = 0
  return res, bool_vec

def best_qp(all_q:np.array, elbow:bool, coord:np.array)->np.array:
  best=np.array([2*pi, 2*pi, 2*pi])
  for i in range(8):
    q0=all_q[0][i]
    q1=all_q[1][i]
    q2=all_q[2][i]
    q0 = change_angle_for_j0(q0)
    q=np.array([q0,q1,q2])
    #print(q)  #to see all the possible solutions
    res, bool_vec = range_check(q)
    if (elbow and q1>0 and q2<0) or (not elbow and q1<0 and q2>0) and res:
    #if res: 
      #print("distances:")
      coord_calc=dir_kin(q)
      #print(coord_calc)
      dist = np.linalg.norm(coord-coord_calc)
      #print(coord)
      #print(coord - coord_calc)
      #print(dist)
      best_dist= np.linalg.norm(coord-dir_kin(best))
      #print(best_dist)
      #print("\n")
      if best_dist > dist:
        best_dist=dist
        best=q
      elif best_dist==dist and np.linagl.norm(best)>np.linalg.norm(q):
        best=q
  res, bool_vec = range_check(best)
  if not res:
    best = out_of_joint_range(all_q, elbow, coord)
  return best

def out_of_joint_range(all_q:np.array, elbow:bool, coord:np.array):
  best=np.array([2*pi, 2*pi, 2*pi])
  high_limits = np.array([5*pi/4, 2.2689, 1.9199])         #limiti
  low_limits = np.array([-pi/4, -0.2618, -1.9199])       #limiti
  for i in range(8):
    q0=all_q[0][i]
    q1=all_q[1][i]
    q2=all_q[2][i]

    q=np.array([q0,q1,q2])
    res, bool_vec = range_check(q)
    if ((elbow and q1>0 and q2<0) or (not elbow and q1<0 and q2>0)) and np.sum(bool_vec) == 2:
    #if np.sum(bool_vec) == 2:
      indx = np.where(bool_vec == 0)[0]
      if q[indx] > 0:
        q[indx] = high_limits[indx]
      else:
        q[indx] = low_limits[indx]
      coord_calc=dir_kin(q)
      dist = np.linalg.norm(coord - coord_calc)
      best_dist= np.linalg.norm(coord-dir_kin(best))
      if best_dist > dist:
        best_dist=dist
        best=q
      elif best_dist==dist and np.linagl.norm(best)>np.linalg.norm(q):
        best=q
  res, bool_vec = range_check(best)
  if res:
    return best
  
  for i in range(8):
    q0=all_q[0][i]
    q1=all_q[1][i]
    q2=all_q[2][i]
    q=np.array([q0,q1,q2])
    print("bad situation: you are out of joint limits")
    res, bool_vec = range_check(q)
    #if (elbow and q1>0 and q2<0) or (not elbow and q1<0 and q2>0) and range_check(q):
    if np.sum(bool_vec) == 1:
      indx = np.where(bool_vec == 0)[0]
      for indx_v in indx:
        if q[indx_v] > 0:
          q[indx_v] = high_limits[indx_v]
        else:
          q[indx_v] = low_limits[indx_v]
      coord_calc=dir_kin(q)
      dist = np.linalg.norm(coord - coord_calc)
      best_dist= np.linalg.norm(coord-dir_kin(best))
      if best_dist > dist:
        best_dist=dist
        best=q
      elif best_dist==dist and np.linalg.norm(best)>np.linalg.norm(q):
        best=q
  res, bool_vec = range_check(best)
  if not res:
    best = np.zeros(3)
  return best

#per non buttare
#########################################################################################
#########################################################################################
#########################################################################################

def inv_kin(coord:np.array)->np.array:
    q0=atan2(coord[1],coord[0])
    c2=(pow(m.sqrt(pow(coord[0],2)+pow(coord[1],2))-a0,2)+pow(coord[2]-d0,2)-pow(l1,2)-pow(l2,2))/(2*l1*l2)
    s2=sqrt(1-pow(c2,2))
    q2=atan2(s2,c2)            
    A=np.matrix([[l1+l2*c2, -l2*s2], [l2*s2, l1+l2*c2]])
    vec=np.matrix([[cos(q0)*coord[0]+sin(q0)*coord[1]-a0], [coord[2]-d0]])
    so=np.matmul(np.linalg.inv(A),vec)
    c1=so[0,0]
    s1=so[1,0]
    q1=atan2(s1,c1)
    q=np.array([q0,q1,q2])
    return q

def inv_kin_total(position:np.array, Rtarg:np.matrix, phase):
    Ptarg = np.column_stack([Rtarg, position.transpose()])
    Ptarg = np.row_stack([Ptarg, np.array([0,0,0,1])])
    if phase:
      dist_ee = np.array([0,0,pirul,1])
    else:
      dist_ee = np.array([0,0,maintenance,1])

    print("dist_ee:\n")
    print(dist_ee)
    pint = np.dot(Ptarg, dist_ee)
    pint = np.array([pint[0,0], pint[0,1], pint[0,2]])
    q=inv_kin(pint)
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    Pint = np.matrix(np.array([[ sin(q2 + q3)*cos(q1), -sin(q1), cos(q2 + q3)*cos(q1),       (cos(q1)*(81*cos(q2 + q3) + 59*cos(q2) + 4))/200],
                               [ sin(q2 + q3)*sin(q1),  cos(q1), cos(q2 + q3)*sin(q1),       (sin(q1)*(81*cos(q2 + q3) + 59*cos(q2) + 4))/200],
                               [        -cos(q2 + q3),        0,         sin(q2 + q3),    (81*sin(q2 + q3))/200 + (59*sin(q2))/200 + 0.048826],
                               [                    0,        0,                    0,                                                      1]]))
    print(Pint)
    Rint = Pint[0:3, 0:3]
    Rsol = np.matmul(Rint.transpose(), Rtarg)
    c5 = Rsol[2,2]
    s5 = m.sqrt(1-c5**2)
    q4 = atan2(-Rsol[1,2], -Rsol[0,2])
    q5 = atan2(s5,c5)
    q6 = atan2(-Rsol[2,1],  Rsol[2,0])
    q = np.array([q1,q2,q3,q4,q5,q6])
    return q
 
######################################################################################
######################################################################################
######################################################################################

## Time scaling
def scale_traj(Ti, lim_vel):
    max_vel = Ti.qd.max()
    min_vel = np.abs(Ti.qd.min())
    if max_vel < min_vel:
        mac_vel = min_vel
    k = math.ceil(max_vel / lim_vel)
    #print(max_vel)

    t = 1000*k
    if isinstance(t, int):
            tscal = 1.0
            ts = np.linspace(0, 1, t)  # normalized time from 0 -> 1
            tv = ts * t
    #print(tv)
    q = np.zeros((t,6))
    qd = np.zeros((t,6))
    qdd = np.zeros((t,6))
    for u in tv:
        u = math.floor(u)
        if u == t:
            u = t-1

        original_indx = u // k
        qd[u] = Ti.qd[original_indx]/k
        qdd[u] = Ti.qdd[original_indx]/(k*k)
        if u == 0:
            q[u] = Ti.q[0]
        elif u%k == 0:
            q[u] = Ti.q[original_indx]
        else:
            q[u] = q[u-1] + (qd[u]+qd[u-1])*0.001/2
    
    ## definition of scaled trajectory
    T_scaled = Trajectory('miatrap', tv, q, qd, qdd)
    T_scaled.plot(True)
    return T_scaled, k


#converti valore per il giunto base nuovo
def change_angle_for_j0(angle):
  if angle < -pi/4:
    return angle + pi*2
  else:
    return angle


# calibrazione
def calibration(psi: np.array, meas: np.array):
  phi = eval_phi(psi)
  pos_nom = eval_pos(psi)
  delta_pos = meas - pos_nom
  pseudo_phi = np.linalg.pinv(phi)
  delta_psi = pseudo_phi*delta_pos
  new_psi = psi + delta_psi
  if delta_psi <= 0.0001:
    return new_psi
  else:
    return calibration(new_psi)


def eval_phi(psi:np.array, q:np.array):
  alpha3 = psi[0]
  a1 = psi[1]
  a2 = psi[2]
  a4 = psi[3]
  d1 = psi[4]
  d4 = psi[5]
  d6 = psi[6]

  q1 = q[0]
  q2 = q[1]
  q3 = q[2]
  q4 = q[3]
  q5 = q[4]
  q6 = q[5]

  alph = np.array([[ sin(q6)*(cos(alpha3)*cos(q4)*sin(q1) - cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(alpha3) + cos(q1)*cos(q4)*sin(alpha3)*sin(q2)*sin(q3)) - cos(q6)*(sin(q5)*(sin(alpha3)*sin(q1) + cos(alpha3)*cos(q1)*cos(q2)*cos(q3) - cos(alpha3)*cos(q1)*sin(q2)*sin(q3)) - cos(q5)*(cos(alpha3)*sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q3)*sin(alpha3)*sin(q4) + cos(q1)*sin(alpha3)*sin(q2)*sin(q3)*sin(q4))),   sin(q6)*(sin(q5)*(sin(alpha3)*sin(q1) + cos(alpha3)*cos(q1)*cos(q2)*cos(q3) - cos(alpha3)*cos(q1)*sin(q2)*sin(q3)) - cos(q5)*(cos(alpha3)*sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q3)*sin(alpha3)*sin(q4) + cos(q1)*sin(alpha3)*sin(q2)*sin(q3)*sin(q4))) + cos(q6)*(cos(alpha3)*cos(q4)*sin(q1) - cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(alpha3) + cos(q1)*cos(q4)*sin(alpha3)*sin(q2)*sin(q3)), - cos(q5)*(sin(alpha3)*sin(q1) + cos(alpha3)*cos(q1)*cos(q2)*cos(q3) - cos(alpha3)*cos(q1)*sin(q2)*sin(q3)) - sin(q5)*(cos(alpha3)*sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q3)*sin(alpha3)*sin(q4) + cos(q1)*sin(alpha3)*sin(q2)*sin(q3)*sin(q4)), a4*cos(alpha3)*sin(q1)*sin(q4) - d4*sin(alpha3)*sin(q1) - d6*cos(q5)*sin(alpha3)*sin(q1) - d4*cos(alpha3)*cos(q1)*cos(q2)*cos(q3) + d4*cos(alpha3)*cos(q1)*sin(q2)*sin(q3) - d6*cos(alpha3)*sin(q1)*sin(q4)*sin(q5) + a4*cos(q1)*sin(alpha3)*sin(q2)*sin(q3)*sin(q4) - d6*cos(alpha3)*cos(q1)*cos(q2)*cos(q3)*cos(q5) - a4*cos(q1)*cos(q2)*cos(q3)*sin(alpha3)*sin(q4) + d6*cos(alpha3)*cos(q1)*cos(q5)*sin(q2)*sin(q3) + d6*cos(q1)*cos(q2)*cos(q3)*sin(alpha3)*sin(q4)*sin(q5) - d6*cos(q1)*sin(alpha3)*sin(q2)*sin(q3)*sin(q4)*sin(q5)],
                   [ cos(q6)*(sin(q5)*(cos(q1)*sin(alpha3) - cos(alpha3)*cos(q2)*cos(q3)*sin(q1) + cos(alpha3)*sin(q1)*sin(q2)*sin(q3)) - cos(q5)*(cos(alpha3)*cos(q1)*sin(q4) + cos(q2)*cos(q3)*sin(alpha3)*sin(q1)*sin(q4) - sin(alpha3)*sin(q1)*sin(q2)*sin(q3)*sin(q4))) - sin(q6)*(cos(alpha3)*cos(q1)*cos(q4) + cos(q2)*cos(q3)*cos(q4)*sin(alpha3)*sin(q1) - cos(q4)*sin(alpha3)*sin(q1)*sin(q2)*sin(q3)), - cos(q6)*(cos(alpha3)*cos(q1)*cos(q4) + cos(q2)*cos(q3)*cos(q4)*sin(alpha3)*sin(q1) - cos(q4)*sin(alpha3)*sin(q1)*sin(q2)*sin(q3)) - sin(q6)*(sin(q5)*(cos(q1)*sin(alpha3) - cos(alpha3)*cos(q2)*cos(q3)*sin(q1) + cos(alpha3)*sin(q1)*sin(q2)*sin(q3)) - cos(q5)*(cos(alpha3)*cos(q1)*sin(q4) + cos(q2)*cos(q3)*sin(alpha3)*sin(q1)*sin(q4) - sin(alpha3)*sin(q1)*sin(q2)*sin(q3)*sin(q4))),   cos(q5)*(cos(q1)*sin(alpha3) - cos(alpha3)*cos(q2)*cos(q3)*sin(q1) + cos(alpha3)*sin(q1)*sin(q2)*sin(q3)) + sin(q5)*(cos(alpha3)*cos(q1)*sin(q4) + cos(q2)*cos(q3)*sin(alpha3)*sin(q1)*sin(q4) - sin(alpha3)*sin(q1)*sin(q2)*sin(q3)*sin(q4)), d4*cos(q1)*sin(alpha3) - a4*cos(alpha3)*cos(q1)*sin(q4) + d6*cos(q1)*cos(q5)*sin(alpha3) - d4*cos(alpha3)*cos(q2)*cos(q3)*sin(q1) + d6*cos(alpha3)*cos(q1)*sin(q4)*sin(q5) + d4*cos(alpha3)*sin(q1)*sin(q2)*sin(q3) - a4*cos(q2)*cos(q3)*sin(alpha3)*sin(q1)*sin(q4) + d6*cos(alpha3)*cos(q5)*sin(q1)*sin(q2)*sin(q3) + a4*sin(alpha3)*sin(q1)*sin(q2)*sin(q3)*sin(q4) - d6*cos(alpha3)*cos(q2)*cos(q3)*cos(q5)*sin(q1) - d6*sin(alpha3)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + d6*cos(q2)*cos(q3)*sin(alpha3)*sin(q1)*sin(q4)*sin(q5)],
                   [                                                                                                                                                                    - sin(q6)*(cos(q2)*cos(q4)*sin(alpha3)*sin(q3) + cos(q3)*cos(q4)*sin(alpha3)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q2)*sin(alpha3)*sin(q3)*sin(q4) + cos(q3)*sin(alpha3)*sin(q2)*sin(q4)) + sin(q2 + q3)*cos(alpha3)*sin(q5)),                                                                                                                                                                        sin(q6)*(cos(q5)*(cos(q2)*sin(alpha3)*sin(q3)*sin(q4) + cos(q3)*sin(alpha3)*sin(q2)*sin(q4)) + sin(q2 + q3)*cos(alpha3)*sin(q5)) - cos(q6)*(cos(q2)*cos(q4)*sin(alpha3)*sin(q3) + cos(q3)*cos(q4)*sin(alpha3)*sin(q2)),                                                                                                                          sin(q5)*(cos(q2)*sin(alpha3)*sin(q3)*sin(q4) + cos(q3)*sin(alpha3)*sin(q2)*sin(q4)) - sin(q2 + q3)*cos(alpha3)*cos(q5),                                                                                                                                                                                                     d6*cos(q2)*sin(alpha3)*sin(q3)*sin(q4)*sin(q5) - d4*cos(alpha3)*cos(q3)*sin(q2) - d6*cos(alpha3)*cos(q2)*cos(q5)*sin(q3) - d6*cos(alpha3)*cos(q3)*cos(q5)*sin(q2) - a4*cos(q2)*sin(alpha3)*sin(q3)*sin(q4) - a4*cos(q3)*sin(alpha3)*sin(q2)*sin(q4) - d4*cos(alpha3)*cos(q2)*sin(q3) + d6*cos(q3)*sin(alpha3)*sin(q2)*sin(q4)*sin(q5)]])
  alph = SO3(alph)

  r_alpha_x = a4*cos(alpha3)*sin(q1)*sin(q4) - d4*sin(alpha3)*sin(q1) - d6*cos(q5)*sin(alpha3)*sin(q1) - d4*cos(alpha3)*cos(q1)*cos(q2)*cos(q3) + d4*cos(alpha3)*cos(q1)*sin(q2)*sin(q3) - d6*cos(alpha3)*sin(q1)*sin(q4)*sin(q5) + a4*cos(q1)*sin(alpha3)*sin(q2)*sin(q3)*sin(q4) - d6*cos(alpha3)*cos(q1)*cos(q2)*cos(q3)*cos(q5) - a4*cos(q1)*cos(q2)*cos(q3)*sin(alpha3)*sin(q4) + d6*cos(alpha3)*cos(q1)*cos(q5)*sin(q2)*sin(q3) + d6*cos(q1)*cos(q2)*cos(q3)*sin(alpha3)*sin(q4)*sin(q5) - d6*cos(q1)*sin(alpha3)*sin(q2)*sin(q3)*sin(q4)*sin(q5)
  r_alpha_y = d4*cos(q1)*sin(alpha3) - a4*cos(alpha3)*cos(q1)*sin(q4) + d6*cos(q1)*cos(q5)*sin(alpha3) - d4*cos(alpha3)*cos(q2)*cos(q3)*sin(q1) + d6*cos(alpha3)*cos(q1)*sin(q4)*sin(q5) + d4*cos(alpha3)*sin(q1)*sin(q2)*sin(q3) - a4*cos(q2)*cos(q3)*sin(alpha3)*sin(q1)*sin(q4) + d6*cos(alpha3)*cos(q5)*sin(q1)*sin(q2)*sin(q3) + a4*sin(alpha3)*sin(q1)*sin(q2)*sin(q3)*sin(q4) - d6*cos(alpha3)*cos(q2)*cos(q3)*cos(q5)*sin(q1) - d6*sin(alpha3)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + d6*cos(q2)*cos(q3)*sin(alpha3)*sin(q1)*sin(q4)*sin(q5)
  r_alpha_z = d6*cos(q2)*sin(alpha3)*sin(q3)*sin(q4)*sin(q5) - d4*cos(alpha3)*cos(q3)*sin(q2) - d6*cos(alpha3)*cos(q2)*cos(q5)*sin(q3) - d6*cos(alpha3)*cos(q3)*cos(q5)*sin(q2) - a4*cos(q2)*sin(alpha3)*sin(q3)*sin(q4) - a4*cos(q3)*sin(alpha3)*sin(q2)*sin(q4) - d4*cos(alpha3)*cos(q2)*sin(q3) + d6*cos(q3)*sin(alpha3)*sin(q2)*sin(q4)*sin(q5)
  r_alpha_rpy = alph.rpy()
  r_alpha = np.array([r_alpha_x, r_alpha_y, r_alpha_z, r_alpha_rpy[0], r_alpha_rpy[1], r_alpha_rpy[2]])

  r_a1_x = cos(q1)
  r_a1_y = sin(q1)
  r_a1_z = 0
  r_a1 = np.array([r_a1_x, r_a1_y, r_a1_z, 0, 0, 0])

  r_a2_x = cos(q1)*cos(q2)
  r_a2_y = cos(q2)*sin(q1)
  r_a2_z = sin(q2)
  r_a2 = np.array([r_a2_x, r_a2_y, r_a2_z, 0, 0, 0])

  r_a4_x = sin(alpha3)*sin(q1)*sin(q4) + cos(q1)*cos(q2)*cos(q4)*sin(q3) + cos(q1)*cos(q3)*cos(q4)*sin(q2) + cos(alpha3)*cos(q1)*cos(q2)*cos(q3)*sin(q4) - cos(alpha3)*cos(q1)*sin(q2)*sin(q3)*sin(q4)
  r_a4_y = cos(q2)*cos(q4)*sin(q1)*sin(q3) - cos(q1)*sin(alpha3)*sin(q4) + cos(q3)*cos(q4)*sin(q1)*sin(q2) + cos(alpha3)*cos(q2)*cos(q3)*sin(q1)*sin(q4) - cos(alpha3)*sin(q1)*sin(q2)*sin(q3)*sin(q4)
  r_a4_z = cos(q4)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*cos(q4) + cos(alpha3)*cos(q2)*sin(q3)*sin(q4) + cos(alpha3)*cos(q3)*sin(q2)*sin(q4)
  r_a4 = np.array([r_a4_x, r_a4_y, r_a4_z, 0, 0, 0])

  r_d1_x = 0
  r_d1_y = 0
  r_d1_z = 1
  r_d1 = np.array([r_d1_x, r_d1_y, r_d1_z, 0, 0, 0])

  r_d4_x = cos(alpha3)*sin(q1) - cos(q1)*cos(q2)*cos(q3)*sin(alpha3) + cos(q1)*sin(alpha3)*sin(q2)*sin(q3)
  r_d4_y = sin(alpha3)*sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(alpha3)*sin(q1) - cos(alpha3)*cos(q1)
  r_d4_z = - cos(q2)*sin(alpha3)*sin(q3) - cos(q3)*sin(alpha3)*sin(q2)
  r_d4 = np.array([r_d4_x, r_d4_y, r_d4_z, 0, 0, 0])

  r_d6_x = cos(alpha3)*cos(q5)*sin(q1) - sin(alpha3)*sin(q1)*sin(q4)*sin(q5) - cos(q1)*cos(q2)*cos(q3)*cos(q5)*sin(alpha3) - cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q5) - cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5) + cos(q1)*cos(q5)*sin(alpha3)*sin(q2)*sin(q3) - cos(alpha3)*cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q5) + cos(alpha3)*cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5)
  r_d6_y = cos(q1)*sin(alpha3)*sin(q4)*sin(q5) - cos(alpha3)*cos(q1)*cos(q5) - cos(q2)*cos(q3)*cos(q5)*sin(alpha3)*sin(q1) - cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q5) - cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5) + cos(q5)*sin(alpha3)*sin(q1)*sin(q2)*sin(q3) - cos(alpha3)*cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q5) + cos(alpha3)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5)
  r_d6_z = cos(q2)*cos(q3)*cos(q4)*sin(q5) - cos(q2)*cos(q5)*sin(alpha3)*sin(q3) - cos(q3)*cos(q5)*sin(alpha3)*sin(q2) - cos(q4)*sin(q2)*sin(q3)*sin(q5) - cos(alpha3)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - cos(alpha3)*cos(q3)*sin(q2)*sin(q4)*sin(q5)
  r_d6= np.array([r_d6_x, r_d6_y, r_d6_z, 0, 0, 0])

  phi = np.column_stack(r_alpha, r_a1, r_a2, r_a4, r_d1, r_d4, r_d6)

  return phi


def eval_pos(psi:np.array, q:np.array):
  alpha3 = psi[0]
  a1 = psi[1]
  a2 = psi[2]
  a4 = psi[3]
  d1 = psi[4]
  d4 = psi[5]
  d6 = psi[6]

  q1 = q[0]
  q2 = q[1]
  q3 = q[2]
  q4 = q[3]
  q5 = q[4]
  q6 = q[5]

  Rot = np.array([[cos(q6)*(sin(q5)*(cos(alpha3)*sin(q1) - cos(q1)*cos(q2)*cos(q3)*sin(alpha3) + cos(q1)*sin(alpha3)*sin(q2)*sin(q3)) + cos(q5)*(sin(q2 + q3)*cos(q1)*cos(q4) + sin(alpha3)*sin(q1)*sin(q4) + cos(alpha3)*cos(q1)*cos(q2)*cos(q3)*sin(q4) - cos(alpha3)*cos(q1)*sin(q2)*sin(q3)*sin(q4))) - sin(q6)*(sin(q2 + q3)*cos(q1)*sin(q4) - cos(q4)*sin(alpha3)*sin(q1) - cos(alpha3)*cos(q1)*cos(q2)*cos(q3)*cos(q4) + cos(alpha3)*cos(q1)*cos(q4)*sin(q2)*sin(q3)), - sin(q6)*(sin(q5)*(cos(alpha3)*sin(q1) - cos(q1)*cos(q2)*cos(q3)*sin(alpha3) + cos(q1)*sin(alpha3)*sin(q2)*sin(q3)) + cos(q5)*(sin(q2 + q3)*cos(q1)*cos(q4) + sin(alpha3)*sin(q1)*sin(q4) + cos(alpha3)*cos(q1)*cos(q2)*cos(q3)*sin(q4) - cos(alpha3)*cos(q1)*sin(q2)*sin(q3)*sin(q4))) - cos(q6)*(sin(q2 + q3)*cos(q1)*sin(q4) - cos(q4)*sin(alpha3)*sin(q1) - cos(alpha3)*cos(q1)*cos(q2)*cos(q3)*cos(q4) + cos(alpha3)*cos(q1)*cos(q4)*sin(q2)*sin(q3)),   cos(q5)*(cos(alpha3)*sin(q1) - cos(q1)*cos(q2)*cos(q3)*sin(alpha3) + cos(q1)*sin(alpha3)*sin(q2)*sin(q3)) - sin(q5)*(sin(q2 + q3)*cos(q1)*cos(q4) + sin(alpha3)*sin(q1)*sin(q4) + cos(alpha3)*cos(q1)*cos(q2)*cos(q3)*sin(q4) - cos(alpha3)*cos(q1)*sin(q2)*sin(q3)*sin(q4))],
                    [- cos(q6)*(sin(q5)*(cos(alpha3)*cos(q1) + cos(q2)*cos(q3)*sin(alpha3)*sin(q1) - sin(alpha3)*sin(q1)*sin(q2)*sin(q3)) - cos(q5)*(sin(q2 + q3)*cos(q4)*sin(q1) - cos(q1)*sin(alpha3)*sin(q4) + cos(alpha3)*cos(q2)*cos(q3)*sin(q1)*sin(q4) - cos(alpha3)*sin(q1)*sin(q2)*sin(q3)*sin(q4))) - sin(q6)*(sin(q2 + q3)*sin(q1)*sin(q4) + cos(q1)*cos(q4)*sin(alpha3) - cos(alpha3)*cos(q2)*cos(q3)*cos(q4)*sin(q1) + cos(alpha3)*cos(q4)*sin(q1)*sin(q2)*sin(q3)),   sin(q6)*(sin(q5)*(cos(alpha3)*cos(q1) + cos(q2)*cos(q3)*sin(alpha3)*sin(q1) - sin(alpha3)*sin(q1)*sin(q2)*sin(q3)) - cos(q5)*(sin(q2 + q3)*cos(q4)*sin(q1) - cos(q1)*sin(alpha3)*sin(q4) + cos(alpha3)*cos(q2)*cos(q3)*sin(q1)*sin(q4) - cos(alpha3)*sin(q1)*sin(q2)*sin(q3)*sin(q4))) - cos(q6)*(sin(q2 + q3)*sin(q1)*sin(q4) + cos(q1)*cos(q4)*sin(alpha3) - cos(alpha3)*cos(q2)*cos(q3)*cos(q4)*sin(q1) + cos(alpha3)*cos(q4)*sin(q1)*sin(q2)*sin(q3)), - cos(q5)*(cos(alpha3)*cos(q1) + cos(q2)*cos(q3)*sin(alpha3)*sin(q1) - sin(alpha3)*sin(q1)*sin(q2)*sin(q3)) - sin(q5)*(sin(q2 + q3)*cos(q4)*sin(q1) - cos(q1)*sin(alpha3)*sin(q4) + cos(alpha3)*cos(q2)*cos(q3)*sin(q1)*sin(q4) - cos(alpha3)*sin(q1)*sin(q2)*sin(q3)*sin(q4))], 
                    [cos(q6)*(cos(q5)*(cos(q4)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*cos(q4) + cos(alpha3)*cos(q2)*sin(q3)*sin(q4) + cos(alpha3)*cos(q3)*sin(q2)*sin(q4)) - sin(q2 + q3)*sin(alpha3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q3)*sin(q4) - sin(q2)*sin(q3)*sin(q4) + cos(alpha3)*cos(q2)*cos(q4)*sin(q3) + cos(alpha3)*cos(q3)*cos(q4)*sin(q2)),                                                                                                                              cos(q6)*(cos(q2)*cos(q3)*sin(q4) - sin(q2)*sin(q3)*sin(q4) + cos(alpha3)*cos(q2)*cos(q4)*sin(q3) + cos(alpha3)*cos(q3)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*cos(q4) + cos(alpha3)*cos(q2)*sin(q3)*sin(q4) + cos(alpha3)*cos(q3)*sin(q2)*sin(q4)) - sin(q2 + q3)*sin(alpha3)*sin(q5)),                                                                                                   - sin(q5)*(cos(q4)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*cos(q4) + cos(alpha3)*cos(q2)*sin(q3)*sin(q4) + cos(alpha3)*cos(q3)*sin(q2)*sin(q4)) - sin(q2 + q3)*cos(q5)*sin(alpha3)]])
  Rot = SO3(Rot)
  r_rpy = Rot.rpy()
  r_x = a1*cos(q1) + a2*cos(q1)*cos(q2) + d4*cos(alpha3)*sin(q1) + d6*cos(alpha3)*cos(q5)*sin(q1) + a4*sin(alpha3)*sin(q1)*sin(q4) - d4*cos(q1)*cos(q2)*cos(q3)*sin(alpha3) + a4*cos(q1)*cos(q2)*cos(q4)*sin(q3) + a4*cos(q1)*cos(q3)*cos(q4)*sin(q2) + d4*cos(q1)*sin(alpha3)*sin(q2)*sin(q3) - d6*sin(alpha3)*sin(q1)*sin(q4)*sin(q5) - a4*cos(alpha3)*cos(q1)*sin(q2)*sin(q3)*sin(q4) - d6*cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q5) - d6*cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5) + d6*cos(q1)*cos(q5)*sin(alpha3)*sin(q2)*sin(q3) + a4*cos(alpha3)*cos(q1)*cos(q2)*cos(q3)*sin(q4) - d6*cos(q1)*cos(q2)*cos(q3)*cos(q5)*sin(alpha3) - d6*cos(alpha3)*cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q5) + d6*cos(alpha3)*cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5)
  r_y = a1*sin(q1) - d4*cos(alpha3)*cos(q1) + a2*cos(q2)*sin(q1) - d6*cos(alpha3)*cos(q1)*cos(q5) - a4*cos(q1)*sin(alpha3)*sin(q4) - d4*cos(q2)*cos(q3)*sin(alpha3)*sin(q1) + a4*cos(q2)*cos(q4)*sin(q1)*sin(q3) + a4*cos(q3)*cos(q4)*sin(q1)*sin(q2) + d6*cos(q1)*sin(alpha3)*sin(q4)*sin(q5) + d4*sin(alpha3)*sin(q1)*sin(q2)*sin(q3) - a4*cos(alpha3)*sin(q1)*sin(q2)*sin(q3)*sin(q4) - d6*cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q5) - d6*cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5) + d6*cos(q5)*sin(alpha3)*sin(q1)*sin(q2)*sin(q3) + a4*cos(alpha3)*cos(q2)*cos(q3)*sin(q1)*sin(q4) - d6*cos(q2)*cos(q3)*cos(q5)*sin(alpha3)*sin(q1) - d6*cos(alpha3)*cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q5) + d6*cos(alpha3)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5)
  r_z = d1 + a2*sin(q2) - a4*cos(q2)*cos(q3)*cos(q4) - d4*cos(q2)*sin(alpha3)*sin(q3) - d4*cos(q3)*sin(alpha3)*sin(q2) + a4*cos(q4)*sin(q2)*sin(q3) + a4*cos(alpha3)*cos(q2)*sin(q3)*sin(q4) + a4*cos(alpha3)*cos(q3)*sin(q2)*sin(q4) + d6*cos(q2)*cos(q3)*cos(q4)*sin(q5) - d6*cos(q2)*cos(q5)*sin(alpha3)*sin(q3) - d6*cos(q3)*cos(q5)*sin(alpha3)*sin(q2) - d6*cos(q4)*sin(q2)*sin(q3)*sin(q5) - d6*cos(alpha3)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - d6*cos(alpha3)*cos(q3)*sin(q2)*sin(q4)*sin(q5)
  r = np.array([r_x, r_y, r_z, r_rpy[0], r_rpy[1], r_rpy[2]])
  
  return r


# per andare dritti
def go_straight(task:bool, elbow:bool, q_pres:np.array, direct: string, steps:int, distance, in_final_frame:bool = False):
    q_des = []
    if direct == 'l':    # left
      s = 1
      pos_change = -distance/steps
    elif direct == 'r':  # right
      s = 1
      pos_change = distance/steps
    elif direct == 'u':  # up
      s = 2
      pos_change = distance/steps
    elif direct == 'd':  # down
      s = 2
      pos_change = -distance/steps
    elif direct == 'f':  # front
      s = 0
      pos_change = distance/steps
    elif direct == 'b':  # back
      s = 0
      pos_change = -distance/steps

    rot, coord = dir_kin_total_with_rot(q_pres, task)
    for i in range(steps):
      pos_change_vec = np.zeros(3)
      pos_change_vec[s] = pos_change
      pos_delta = np.array([pos_change_vec])
      if in_final_frame:
        pos_delta = np.matmul(rot,pos_change_vec)
        #print("vettori variazione pos:\n")
        #print(pos_change_vec)
        #print(pos_delta)
        #print(coord)
        #print("\n")
      coord = np.array([coord[0] + pos_delta[0,0], coord[1] + pos_delta[0,1], coord[2] + pos_delta[0,2]])
      #print(coord)
      prec_qfin = q_pres[3:]
      q_des.append(inv_kin_total_bis(coord, rot, task, elbow, prec_qfin))
    return q_des
        

# for maintenance - motion from Rtag
def move_from_rtag(cod, q_pres, steps):
  if cod == 'main':
    return go_straight(False, True, q_pres, 'r', steps, 0.084, True)
  
  
# per ruotare
def rotate(task:bool, elbow:bool, q_pres:np.array, direct: string, steps:int, distance, in_final_frame:bool = False):
    q_des = []
    if direct == 'r_cw':    # roll cw
      s = 0
      pos_change = distance/steps
    elif direct == 'r_ccw':  # roll ccw
      s = 0
      pos_change = -distance/steps
    elif direct == 'p_cw':  # pitch cw
      s = 1
      pos_change = distance/steps
    elif direct == 'p_ccw':  # pitch ccw
      s = 1
      pos_change = -distance/steps
    elif direct == 'y_cw':  # yaw cw
      s = 2
      pos_change = distance/steps
    elif direct == 'y_ccw':  # yaw ccw
      s = 2
      pos_change = -distance/steps

    rot, coord = dir_kin_total_with_rot(q_pres, task)
    print("partiamo da: ")
    print(coord)
    print("\n")
    for i in range(steps):
      rot_change_vec = np.zeros(3)
      rot_change_vec[s] = pos_change
      rot_delta_so3 = SO3.RPY(rot_change_vec[0], rot_change_vec[1], rot_change_vec[2])
      rot_delta = rot_delta_so3.R
      '''
      if in_final_frame:
        pos_delta = np.matmul(rot,pos_change_vec)
        #print("vettori variazione pos:\n")
        #print(pos_change_vec)
        #print(pos_delta)
        #print(coord)
        #print("\n")
      '''
      rot = rot*rot_delta
      
      #print(coord)
      prec_qfin = q_pres[3:]
      q_des.append(inv_kin_total_bis(coord, rot, task, elbow, prec_qfin))
    return q_des 
