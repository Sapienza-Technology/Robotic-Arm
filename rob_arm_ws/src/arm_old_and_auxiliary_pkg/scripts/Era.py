#!/usr/bin/env python
"""
@author: Alessio Sfregola
@author: Antonio Bozza
@author: Saverio Taliani
@author: Altri
"""

# all parameters are in SI units: m, radians, kg, kg.m2, N.m, N.m.s etc.

# from math import pi
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
from spatialmath import base
from math import sin, cos, pow, atan2, sqrt, pi 
import math as m


class Era(DHRobot):
    """
    Class that models a Era manipulator

    :param symbolic: use symbolic constants
    :type symbolic: bool

    ``Era()`` is an object which models a Unimation Era robot and
    describes its kinematic and dynamic characteristics using standard DH
    conventions.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.DH.Era()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration
    - qs, arm is stretched out in the x-direction
    - qn, arm is at a nominal non-singular configuration

    .. note::
        - SI units are used.
        - The model includes armature inertia and gear ratios.
        - The value of m1 is given as 0 here.  Armstrong found no value for it
          and it does not appear in the equation for tau1 after the
          substituion is made to inertia about link frame rather than COG
          frame.
        - Gravity load torque is the motor torque necessary to keep the joint
          static, and is thus -ve of the gravity caused torque.

    :references:
        - "A search for consensus among model parameters reported for the PUMA
          560 robot", P. Corke and B. Armstrong-Helouvry,
          Proc. IEEE Int. Conf. Robotics and Automation, (San Diego),
          pp. 1608-1613, May 1994. (for kinematic and dynamic parameters)
        - "A combined optimization method for solving the inverse kinematics
          problem", Wang & Chen, IEEE Trans. RA 7(4) 1991 pp 489-.
          (for joint angle limits)
        - https://github.com/4rtur1t0/ARTE/blob/master/robots/UNIMATE/Era/parameters.m

    .. codeauthor:: Peter Corke
    """  # noqa

    def __init__(self, symbolic=False):

        if symbolic:
            import spatialmath.base.symbolic as sym

            zero = sym.zero()
            pi = sym.pi()
        else:
            from math import pi

            zero = 0.0

        deg = pi / 180

        base = 0 # from mounting surface to shoulder axis

        L = [
            RevoluteDH(
                d=0.0655,  # link length (Dennavit-Hartenberg notation)
                a=0.0100,  # link offset (Dennavit-Hartenberg notation)
                alpha= pi / 2,  # link twist (Dennavit-Hartenberg notation)
                I=[0, 0.35, 0, 0, 0, 0],
                # inertia tensor of link with respect to
                # center of mass I = [L_xx, L_yy, L_zz,
                # L_xy, L_yz, L_xz]
                r=[0, 0, 0],
                # distance of ith origin to center of mass [x,y,z]
                # in link reference frame
                m=0,  # mass of link
                Jm=200e-6,  # actuator inertia
                G=-62.6111,  # gear ratio
                B=1.48e-3,  # actuator viscous friction coefficient (measured
                # at the motor)
                Tc=[0.395, -0.435],
                # actuator Coulomb friction coefficient for
                # direction [-,+] (measured at the motor)
                qlim=[- 180 * deg, 180 * deg],  # minimum and maximum joint angle
            ),
            RevoluteDH(
                d=0,
                a=0.2950,
                alpha=zero,
                offset=pi / 2,
                I=[0.13, 0.524, 0.539, 0, 0, 0],
                r=[-0.3638, 0.006, 0.2275],
                m=17.4,
                Jm=200e-6,
                G=107.815,
                B=0.817e-3,
                Tc=[0.126, -0.071],
                qlim=[-110 * deg, 110 * deg],  # qlim=[-45*deg, 225*deg]
            ),
            RevoluteDH(
                d=zero,
                a=zero,
                alpha= -pi / 2,
                offset= -pi / 2,
                I=[0.066, 0.086, 0.0125, 0, 0, 0],
                r=[-0.0203, -0.0141, 0.070],
                m=4.8,
                Jm=200e-6,
                G=-53.7063,
                B=1.38e-3,
                Tc=[0.132, -0.105],
                qlim=[-135 * deg, 135 * deg],  # qlim=[-225*deg, 45*deg]
            ),
            RevoluteDH(
                d=0.3610,
                a=0,
                alpha=pi / 2,
                I=[1.8e-3, 1.3e-3, 1.8e-3, 0, 0, 0],
                r=[0, 0.019, 0],
                m=0.82,
                Jm=33e-6,
                G=76.0364,
                B=71.2e-6,
                Tc=[11.2e-3, -16.9e-3],
                qlim=[-266 * deg, 266 * deg],  # qlim=[-110*deg, 170*deg]
            ),
            RevoluteDH(
                d=0,
                a=0,
                alpha=-pi / 2,
                I=[0.3e-3, 0.4e-3, 0.3e-3, 0, 0, 0],
                r=[0, 0, 0],
                m=0.34,
                Jm=33e-6,
                G=71.923,
                B=82.6e-6,
                Tc=[9.26e-3, -14.5e-3],
                qlim=[-100 * deg, 100 * deg],
            ),
            RevoluteDH(
                d=0.1210,
                a=0,
                alpha=zero,
                I=[0.15e-3, 0.15e-3, 0.04e-3, 0, 0, 0],
                r=[0, 0, 0.032],
                m=0.09,
                Jm=33e-6,
                G=76.686,
                B=36.7e-6,
                Tc=[3.96e-3, -10.5e-3],
                qlim=[-266 * deg, 266 * deg],
            ),
        ]

        super().__init__(
            L,
            name="Era",
            manufacturer="TechTeam",
            keywords=("dynamics", "symbolic", "mesh"),
            symbolic=symbolic,
            meshdir="meshes/Era",
        )

        self.qr = np.array([0, pi / 2, -pi / 2, 0, 0, 0])
        self.qz = np.zeros(6)

        # nominal table top picking pose
        self.qn = np.array([0, pi / 4, pi, 0, pi / 4, 0])

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
        self.addconfiguration("qn", self.qn)

        # straight and horizontal
        self.addconfiguration_attr("qs", np.array([0, 0, -pi / 2, 0, 0, 0]))

    def ikine_a(self, T:SE3, config="lun"):
        """
        Analytic inverse kinematic solution

        :param T: end-effector pose
        :type T: SE3
        :param config: arm configuration, defaults to "lun"
        :type config: str, optional
        :return: joint angle vector in radians
        :rtype: ndarray(6)

        ``robot.ikine_a(T, config)`` is the joint angle vector which achieves the
        end-effector pose ``T```.  The configuration string selects the specific
        solution and is a sting comprising the following letters:

        ======   ==============================================
        Letter   Meaning
        ======   ==============================================
        l        Choose the left-handed configuration
        r        Choose the right-handed configuration
        u        Choose the elbow up configuration
        d        Choose the elbow down configuration
        n        Choose the wrist not-flipped configuration
        f        Choose the wrist flipped configuration
        ======   ==============================================


        :reference:
            - Inverse kinematics for a Era,
              Paul and Zhang,
              The International Journal of Robotics Research,
              Vol. 5, No. 2, Summer 1986, p. 32-44

        :author: based on MATLAB code by Robert Biro with Gary Von McMurray,
            GTRI/ATRP/IIMB, Georgia Institute of Technology, 2/13/95

        """
        #####################################################################
        ## robot constants ###################################################
        a0=0.0100
        d0=0.0655
        l1=0.2950
        l2=0.3610
        pirul = -0.145
        maintenance = -0.121
        #####################################################################


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

        def ik3(Ptarg:np.matrix, phase, elbow:bool, prec_qfin:np.array):
            Rtarg = Ptarg[0:3,0:3]
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
            #print(self.fkine(q))
            #print("dir kin voluta:")
            #print(pint)
            #print("\n")
            q1=q[0]
            q2=q[1]
            q3=q[2]
            Rint = np.matrix(np.array([[ sin(q2 + q3)*cos(q1), -sin(q1), cos(q2 + q3)*cos(q1)],
                                       [ sin(q2 + q3)*sin(q1),  cos(q1), cos(q2 + q3)*sin(q1)],
                                       [        -cos(q2 + q3),        0,         sin(q2 + q3)]]))
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
                    coord_calc=self.fkine(q)
                    #print(coord_calc)
                    dist = np.linalg.norm(coord-coord_calc)
                    #print(coord)
                    #print(coord - coord_calc)
                    #print(dist)
                    best_dist= np.linalg.norm(coord-self.fkine(best))
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
        
        #converti valore per il giunto base nuovo
        def change_angle_for_j0(angle):
            if angle < -pi/4:
                return angle + pi*2
            else:
                return angle


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
                    coord_calc=self.fkine(q)
                    dist = np.linalg.norm(coord - coord_calc)
                    best_dist= np.linalg.norm(coord-self.fkine(best))
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
                    coord_calc=self.fkine(q)
                    dist = np.linalg.norm(coord - coord_calc)
                    best_dist= np.linalg.norm(coord-self.fkine(best))
                    if best_dist > dist:
                        best_dist=dist
                        best=q
                    elif best_dist==dist and np.linalg.norm(best)>np.linalg.norm(q):
                        best=q
            res, bool_vec = range_check(best)
            if not res:
                best = np.zeros(3)
            return best


        Ptarg = T.data
        return self.ikine_6s(T, config, ik3)


if __name__ == "__main__":  # pragma nocover

    puma = Era(symbolic=False)
    print(puma)
    print(puma.dynamics())
    # T = puma.fkine(puma.qn)
    # print(puma.ikine_a(T, 'lu').q)
    # print(puma.ikine_a(T, 'ru').q)
    # print(puma.ikine_a(T, 'ld').q)
    # print(puma.ikine_a(T, 'rd').q)

    # puma.plot(puma.qz)
