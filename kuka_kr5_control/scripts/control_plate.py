#     * States x:
#     alpha : Angle around link4
#     dalpha: angular vel around link 4
#     beta  : Angle around link5
#     dbeta : angular vel around link5
#     x     : ball x pos relative to plate
#     dx    : ball vel x
#     y     : ball y pos relative to plate
#     dy    : ball vel y

#     * Inputs u (torques of link4, link5):
#     MR = M4
#     MP = M5

#     * non linear control model:
#     [dalpha       x2
#     ddalpha       f1(x, u)
#     dbeta         x4
#     ddbeta     =  f2(x, u)
#     dx            x6
#     ddx           f3(x, u)
#     dy            x8
#     ddy]          f4(x, u)

#     * linear control model:
#    dx = Ax+Bu
#    y =  Cx = [x; y]
#        0  1  0  0  0  0  0  0        0  0
#        0  -b 0  0  0  0  -c 0        h  0
#        0  0  0  1  0  0  0  0        0  0        0 0 0 0 1 0 0 0
#   A =  0  0  0 -d -f  0  0  0    B = 0  i    C = 0 0 0 0 0 0 1 0
#        0  0  0  0  0  0  1  0        0  0
#        0  0  -a -e -g 0  0  0        0  j
#        0  0  0  0  0  0  0  0        0  0
#        -a -b 0  0  0  0  -g 1        k  0

#    with constants:
#    a = (g m*r^2)/( m r^2 + TB)
#    b = (dR r) / (TP,xP)  <<TODO (I do not have a Rahmen!
#    c = (g m)  / (TP,xP)
#    h = (g m r)/ (TP,xP)
#    d = (dP )  / (TP,yP)
#    e = (dP r) / (TP,yP)
#    f = (g m)  / (TP,yP)
#    g = (g m r)/ (TP,yP)

#    h = 1      / (TP,xP)
#    i = 1      / (TP,yP)
#    j = r      / (TP,yP)
#    k = r      / (TP,xP)

#    description of parameters:
#    name            value    unit       description
#    m               0.056    kg         ball mass
#    g               9.81     kg/m^2     earth force
#    r               0.032    m          ball radius
#    dP              0        Nm s       damping Plate
#    dR              0.1835   Nm s       damping Rahmen
#    TB              0.000038 kg m^2     ball inertia (ixx=iyy=izz)
#    TP,xP           0.0138   kg m^2     Plate inertia (ixx value in gazebo)
#    TP,yP           0.0138   kg m^2     Plate inertia (iyy)

# Using these values and linearizing around x0 = [0...0], u0 = [0, 0]
# A =
#[[0 1 0 0 0 0 0 0]
# [0 -0.425507246376812 0 0 0 0 -39.8086956521739 0]
# [0 0 0 1 0 0 0 0]
# [0 0 0 0 -39.8086956521739 0 0 0]
# [0 0 0 0 0 0 1 0]
# [0 0 -5.90015774458802 0 -1.27387826086957 0 0 0]
# [0 0 0 0 0 0 0 0]
# [-5.90015774458802 -0.425507246376812 0 0 0 0 -1.27387826086957 1]]

# B=
#[[0 0]
# [72.4637681159420 0]
# [0 0]
# [0 72.4637681159420]
# [0 0]
# [0 2.31884057971015]
# [0 0]
# [2.31884057971015 0]]


#!/usr/bin/env python

import sympy as sp  ## sudo apt-get install python-sympy
import numpy as np
import control as co # pip install control
import slycot            # pip install   slycot


# Do all symbolically first:
g, m, r, dR, dP, TP_xP, TB, TP_yP = sp.symbols('g, m, r, dR, dP, TP_xP, TB, TP_yP')

# constants:
a = (g*m*r**2)/(m*r**2+ TB)
b = (dR) / (TP_xP)  #<<TODO (I do not have a Rahmen!
c = (g*m)  / (TP_xP)
h = (g*m*r)/ (TP_xP)
d = (dP )  / (TP_yP)
e = (dP*r) / (TP_yP)
f = (g*m)  / (TP_yP)
gg = (g*m*r)/ (TP_xP)

h = 1      / (TP_xP)
i = 1      / (TP_yP)
j = r      / (TP_yP)
k = r      / (TP_xP)

# Matrices:
A = sp.Matrix([[ 0,  1,  0,  0,  0,  0,  0,  0],
                [0,  -b,  0,  0,  0,  0,  -c, 0],
                [0,  0,  0,  1,  0,  0,  0,  0],
                [0,  0,  0, -d, -f,  0,  0,  0],
                [0,  0,  0,  0,  0,  1,  0,  0],
                [0,  0,  -a, -e, -gg, 0,  0,  0],
                [0,  0,  0,  0,  0,  0,  0,  1],
                [-a, -b, 0,  0,  0,  0,  -gg, 0]
           ])

B = sp.Matrix([ [0, 0],
                [h,  0],
                [0,  0 ],
                [0,  i],
                [0,  0],
                [0,  j],
                [0,  0],
                [k,  0]
          ])

C = sp.Matrix([[0, 0, 0, 0, 1, 0, 0, 0],
               [0, 0, 0, 0, 0, 0, 1, 0]])

# constant_values
m_    =           0.056    #kg         ball mass
g_    =           9.81     #kg/m^2     earth force
r_    =           0.032    #m          ball radius
dP_   =           0        #Nm s       damping Plate
dR_   =           0.1835   #Nm s       damping Rahmen
TB_   =           0.000038 #kg m^2     ball inertia (ixx=iyy=izz)
TP_xP_=           0.0138   #kg m^2     Plate inertia (ixx value in gazebo)
TP_yP_=           0.0138   #kg m^2     Plate inertia (iyy)

# substitute values in matrices:
subs_vec = [(g, g_), (m, m_), (r, r_), (dP, dP_), (dR, dR_), (TB, TB_),
            (TP_xP, TP_xP_), (TP_yP, TP_yP_)]

A = A.subs(subs_vec)
B = B.subs(subs_vec)

print "A:="
print np.matrix(A)

print "B:="
print np.matrix(B)

A_eig = list(A.eigenvals().keys())

for i in range(0, len(A_eig)):
  A_eig[i]=sp.N(A_eig[i])

print "A_eig:="
print np.matrix(A_eig)

# Controllability
control_matrix = co.ctrb(A, B)
control_matrix = sp.Matrix(control_matrix)

# Observability:
observe_matrix = co.obsv(A, C)
observe_matrix = sp.Matrix(observe_matrix)

print "Rank of control_matrix: "+str(control_matrix.rank())
print "Rank of observe_matrix: "+str(observe_matrix.rank()) ## tODO this is a problem!?

# pole wish position:
poles_wish = [-6, -6, -6, -6, -6, -6, -6, -6]


A = np.matrix(A)
B = np.matrix(B)
C = np.matrix(C)
A = A.astype(float)
B = B.astype(float)
C = C.astype(float)

R = co.place(A, B, poles_wish)

print "R:="
print np.matrix(R)

# check it:
A_neu = A-np.dot(B,R)

print "A-BR="
print np.matrix(A_neu)

print "eig(A-BR)="
print (np.linalg.eig(A_neu)[0])

# filter matrix F = -(C(A-BR)^-1)B)^-1
F = np.linalg.inv(np.dot(C, np.dot(np.linalg.inv(A_neu), B)))

print "F:"
print F

# u = Fw - Rx

w=[0;0]

