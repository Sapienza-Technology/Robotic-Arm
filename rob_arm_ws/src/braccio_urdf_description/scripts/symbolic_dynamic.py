import numpy as np
import spatialmath as sm
import sympy as sp
from sympy import pi
from auxiliary_symbolic_dynamic import DH_for_dynamic, DH_for_dynamic_complete



g0 = sp.Symbol('g0', real=True)
# INPUT VARIABLES % change these values
n = 6;                          #specify the number of joints CHANGE THIS
sigma = [0,0,0,0,0,0];                # put zero for revolute, 1 for prismatic CHANGE THIS

# we write the CoM vectors
r = sp.Matrix(sp.symbols('r(1:4)(1:'+str(n+1)+')', real=True)).reshape(3, n)
m = sp.Matrix(sp.symbols('m1:'+str(n+1), real=True)).reshape(n, 1)

# change the gravity part HERE: it has to be consistent with the zero frame
gravity = sp.Matrix([0, 0, -g0]).reshape(3, 1)  # % WARNING!!!!

# Inertia matrices
IGen = sp.Matrix.ones(3, 3*n)
for i in range(0,n):
    IGen[0:3,3*i:3*(i+1)] = sp.Matrix(sp.symbols(['Ixx'+str(i),'Ixy'+str(i),'Ixz'+str(i),
                                                'Iyx'+str(i),'Iyy'+str(i),'Iyz'+str(i),
                                                'Izx'+str(i),'Izy'+str(i),'Izz'+str(i)] ,real=True)).reshape(3, 3)


# we write the joint vars
t, L, L1, L2, L3, dc1, dc2, dc3, dc4, h, a1, a2, d4, d6, d1, grip_x, grip_z = sp.symbols('t,L,L1,L2,L3,dc1,dc2,dc3,dc4,h,a1,a2,d4,d6,d1,grip_x,grip_z', real=True)
# specify the CoM vectors that you know
#r = subs(r,r(1,:),[-0.25,0,0])
#r = subs(r,r(2,:),[-0.25,0,0])
#r = subs(r,r(2,:),[-L/2,0,0])
#r = subs(r,r(3,:),[dc3-L3,0,0])
#r = subs(r,r(3,:),[dc3-0.18,0,0])
q = sp.Matrix(sp.symbols('q1:'+str(n+1), real=True)).reshape(n, 1)
q_dot = m = sp.Matrix(sp.symbols('q_d1:'+str(n+1), real=True)).reshape(n, 1)

# we specify the DH_table of the robot CHANGE THIS 
DH_table = sp.Matrix([[ pi/2,  a1,     d1,   q[0]     ],
                    [    0,  a2,      0,   q[1]+pi/2], 
                    [-pi/2,   0,      0,   q[2]-pi/2],
                    [ pi/2,   0,     d4,   q[3]],
                    [-pi/2,   0,      0,   q[4]], 
                    [    0,   0,     d6,   q[5]]])

#%% we find the M matrix
# here the sequence of the T matrices at every step (between the frames)
A = DH_for_dynamic(DH_table)
T = sp.Matrix(sp.symbols('T1:'+str(n+1), real=True)).reshape(n, 1)

# we declare and initialize the vectors for the velocities
omega = sp.Matrix(sp.symbols('omega(1:4)(1:'+str(n+2)+')', real=True)).reshape(3, n+1)
v = sp.Matrix(sp.symbols('v(1:4)(1:'+str(n+2)+')', real=True)).reshape(3, n+1)
omega[:,0] = sp.Matrix([0,0,0]).reshape(3,1)
v[:,0] = sp.Matrix([0,0,0]).reshape(3,1)

# the iterative moving frames algorithm 
for k in range(2,n+2):
    pCoM = r[:,k-2].transpose()
    d = A[0:3, 4*(k-1):4*(k)-1].transpose()@A[0:3, 4*(k-1)]

    print(omega[:,k-1])
    print(pCoM)
    
    omega[:,k-1] = A[0:3, 4*(k-1):4*(k)-1].transpose()@(omega[:,k-2]+(1-sigma[k-2])*q_dot[k-2]*sp.Matrix([0,0,1]).reshape(3,1))
    v[:,k-1] = A[0:3,4*(k-1):4*k-1].transpose()@(v[:,k-2]+sigma[k-2]*q_dot[k-2]*sp.Matrix([0,0,1]).reshape(3,1))+omega[:,k-1].cross(d)
    vck = v[:,k-1]+omega[:,k-1].cross(pCoM)
    
    T[k-2] = 0.5*m[k-2]*vck.transpose()@vck+0.5*omega[:,k-1].transpose()@IGen[:,(k-2)*3:(k-1)*3]@omega[:,k-1]
    T[k-2] = sp.simplify(T[k-2])


T = sp.simplify(T)
Tf = sum(T)

M = sp.simplify(sp.hessian(Tf,q_dot), 'steps', 100)



#%% finding coriolis and centrifucal parts
S = sp.Matrix(sp.symbols('S(1:4)(1:'+str(n+1)+')', real=True)).reshape(n, n)
c = sp.Matrix(sp.symbols('c(1:'+str(n+1)+')', real=True)).reshape(n, 1)
for k in range(n):
    Ck = 0.5*(sp.diff(M[:,k],q)+sp.diff(M[:,k],q).transpose()-sp.diff(M,q(k)))
    S[k,:] = q_dot.transpose()*Ck
    c[k] = q_dot.transpose()*Ck*q_dot

c = sp.simplify(c.transpose())
S = sp.simplify(S.transpose())



#%% finding the gravitational part
U = sp.Matrix(sp.symbols('U1:'+str(n+1), real=True)).reshape(n, 1)

# we find the matrices for the frame j seen in the RF0
A_new = DH_for_dynamic_complete(DH_table)

for j in range(2,n+1):
    p_CoM_0 = A_new[0:4,4*(j-1):4*j]@np.vstack((r[j-1,:].transpose(),1))
    U[j-2] = -m(j-2)*gravity.transpose()@p_CoM_0[0:3,0]

U = sp.simplify(U)

g = sp.simplify(sp.diff(sum(U),q).transpose())

print(M)
print(c)
print(g)