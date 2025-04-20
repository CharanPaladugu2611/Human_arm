"""
Created on 10-02-2023
Edited on 12-01-2023
@directoryID: jchen777
@author: jason chen
@assignment: HW5 (code adapted from my own code from HW3 and HW4).
@python ver: 3.9.13
@IDE: Spyder
"""

from sympy import symbols, Matrix, sin, cos, zeros, trigsimp, pprint, pi, shape, diff
from sympy.utilities.lambdify import implemented_function
from sympy import lambdify
import math
import matplotlib.pyplot as plt
import numpy as np

# DH Table
# Initiate symbolic variables from Sympy
alpha,a,d,theta = symbols("alpha,a,d,theta")
theta0,theta1,theta2,theta3,theta4,theta5 = symbols("theta0,theta1,theta2,theta3,theta4,theta5")
thetas = [theta0,theta1,theta2,theta3,theta4,theta5]

DHTable = Matrix([
            [-pi/2,  0,      3,     0],
            [0,      10.25,  0,     theta0 - pi/2],
            [pi/2,  0,      0,     theta1 + pi/2],
            [0,      0,      0.85,     0],
            [-pi/2,  0,      4.17,  theta2],
            [0,      0,      0.38,  -pi/2     ],
            [0,      0.75,   0,     theta3],
            [0,      0.75,   0,     theta4],
            [pi/2,  1.01,   0,     theta5]]) # For index fingerz

numDummies = 3 # 4 dummy frames used

# Formula for calculating transformation matrix
transMat = Matrix([[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta)],
                      [sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta)],
                      [0,sin(alpha),cos(alpha),d],
                      [0,0,0,1]])

listOfMats = [] # Contains the trans matrices linking consecutive joints
transMatDaddy = Matrix([]) # This is to become the FINAL transformation matrix
rows = int(len(DHTable)/4) # loop once for every one set of alpha, a, d, and theta.

for i in range(rows): # Insert unique values from each row of DH Table into formula
    tempalpha = DHTable[i*4+0]
    tempa = DHTable[i*4+1]
    tempd = DHTable[i*4+2]
    temptheta = DHTable[i*4+3]
    tempTransMat = transMat.subs({alpha:tempalpha,a:tempa,d:tempd,theta:temptheta})
    # Put the different versions of the transformation matrices into a list
    #   so that they can be used to construct the Jacobian
    listOfMats.append(tempTransMat)

# condlistOfMats = listOfMats
condlistOfMats = []
for i in range(rows):
    if(i in [0,2,4]):
        condlistOfMats.append(listOfMats[i]*listOfMats[i+1])
    elif(i in [1,3,5]):
        continue
    else:
        condlistOfMats.append(listOfMats[i])

btolT = [] # Base To (each) Link transformation matrices used for Jacobians
for i in range(len(condlistOfMats)): # Calculate final transformation matrix
    if(i==0):
        transMatDaddy = condlistOfMats[i]
    else:
        transMatDaddy *= condlistOfMats[i] # Multiply L --> R, link 0-1 --> 7-8
    btolT.append(transMatDaddy)
    
# Use lambdify because it is faster than subs
lambdifyTMD = lambdify((theta0,theta1,theta2,theta3,theta4,theta5), transMatDaddy, 'sympy')
    
# This loop is to combine the dummy frames with the actual frame of interest,
#   thus maintaining the nice 6x6 matrix.

JacobianMat = zeros(6,rows-numDummies) # linear x, y, and z, and then angular x, y, and z
On = transMatDaddy[0:3,3] # End effector location relative to the base frame

# The final matrix in the list btolT is ignored because that is just On
for i in range(rows-numDummies):
    if(i == 0):
       Oiminus1 = Matrix([[0],[0],[0]]) # The base position vector in base frame
       Ziminus1 = Matrix([[0],[0],[1]]) # Frame relative to base frame
    else:
       Oiminus1 = btolT[i-1][0:3,3]
       Ziminus1 = btolT[i-1][0:3,2]
    Odiff = On - Oiminus1 # Zi-1 x (On - Oi-1)
    # Manual cross product since I could not find it...
    crossP = Matrix([[Ziminus1[1]*Odiff[2] - Ziminus1[2]*Odiff[1]],
                        [Ziminus1[0]*Odiff[2] - Ziminus1[2]*Odiff[0]],
                        [Ziminus1[0]*Odiff[1] - Ziminus1[1]*Odiff[0]]])
    # Fill in Jacobian Matrix, one column vector at a time. 
    # Position vector in top three rows, rotation in bottom three.
    JacobianMat[0:3,i] = crossP
    JacobianMat[3:6,i] = Ziminus1

lambdifyJacob = lambdify((theta0,theta1,theta2,theta3,theta4,theta5), JacobianMat,'sympy')

# printing to make results look more structured
# for i in range(75): print('=', end='')
# print("\nThe Jacobian matrix is:\n")
# A = trigsimp(JacobianMat) # Takes a while to run this
# pprint(A)
for i in range(75): print('=', end='')
print('\n\n')

steps = 300
time = 15 # seconds we want to complete the task by
angV = 2*np.pi/time/4 # angular velocity in rads/sec

# List to hold the resulting q and qdot vector containing the changeable angle and angular 
#   velocity variables.
q = Matrix([[-0.0001],[0.0001],[-0.0001],[-0.0001],[0.0001],[0.0001]])
# q = Matrix([[0.0001],[-pi/2],[-0.0001],[pi/2],[0.0001],[0.0001]])
qdot = zeros(6,1)

# x, y, and z are for plotting
x = []
y = []
z = []
l = [] # This is for testing the code

temp = np.ones([10,1])
pose0 = zeros(3,1) # initial pose, used to calculate the velocity direction

for i in range(0,steps):
    t = i/steps*time # for loop increments from 1 to the total endtime
    # Get the new most recent pose based on FORWARD KINEMATICS
    newpose = lambdifyTMD(q[0],q[1],q[2],q[3],q[4],q[5])
    # VELOCITY in x and z. y vel is 0. 
    xvel = 1*math.cos(angV*t)*angV
    zvel = -0.1*math.sin(angV*t)*angV
    x.append(newpose[0,3])
    y.append(newpose[1,3])
    z.append(newpose[2,3])
    
    Xdot = Matrix([zvel, zvel, 0, 0, 0, 0])
    # tempJacobian = JacobianMat.subs({theta0:q[0],theta1:q[1],theta2:q[2],
    #                   theta3:q[3],theta4:q[4],theta5:q[5]})
    tempJacobian = lambdifyJacob(q[0],q[1],q[2],q[3],q[4],q[5])
    lPseudoInv = (tempJacobian.T*tempJacobian)**-1*tempJacobian.T
    qdot = lPseudoInv*Xdot
    # qdot = tempJacobian**-1*Xdot
    q += qdot*time/steps

# Plotting the data:
fig = plt.figure()
ax = plt.axes(projection="3d") # Defining the axes to plot 3d graphs
ax.plot(x,y,z)
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
# ax.set_xlim(8,0)
# ax.set_ylim(0.2,0.7)
# ax.set_zlim(20.50,21.00)
plt.show()
print("Here is the Jacobian Matrix:")
pprint(trigsimp(JacobianMat))