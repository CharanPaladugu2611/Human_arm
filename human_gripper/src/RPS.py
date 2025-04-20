#!/usr/bin/env python3
"""
Created on 10-02-2023
Edited on 12-09-2023
@directoryID: jchen777
@author: jason chen
@assignment: HW5 (code adapted from my own code from HW3 and HW4).
@python ver: 3.8.10
@IDE: VS Code
"""
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
import sys
import termios
from sympy import symbols, Matrix, sin, cos, zeros, pi, lambdify
import math
import numpy as np
import time

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

steps = 100
timesp = 1 # seconds we want to complete the task by
angV = 2*np.pi/timesp/4 # angular velocity in rads/sec


# q = Matrix([[0.0001],[-pi/2],[-0.0001],[pi/2],[0.0001],[0.0001]])
qdot = zeros(6,1)

temp = np.ones([10,1])
pose0 = zeros(3,1) # initial pose, used to calculate the velocity direction

class RPS(Node):
   
    def __init__(self):
        super().__init__('rps_node')
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.joint_velocity_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 1)
        self.settings = termios.tcgetattr(sys.stdin)
        self.gripangle = float(pi/5*2)
        self.scissorangle = float(pi/5*2)
        self.elbowpitch = float(0)
        self.wristpitch = float(0)
        self.wristroll = float(0)
        self.joint_positions = Float64MultiArray()
        self.qv = []
        self.scale = float(1.25)
        # List to hold the resulting q and qdot vector containing the changeable angle and angular 
        #   velocity variables.
        self.q = Matrix([[0.01],[0.001],[0.001],[0.001],[0.001],[0.001]])

    def pubjoints(self):
        self.joint_positions.data = [self.elbowpitch,self.wristpitch,-self.wristroll,self.gripangle,self.gripangle,-self.gripangle,self.gripangle,
                                self.gripangle,-self.gripangle,self.gripangle,self.gripangle,-self.gripangle,self.gripangle,self.gripangle,
                                -self.gripangle,self.gripangle,self.gripangle]
        self.joint_position_pub.publish(self.joint_positions)
    
    def pubjoints_scissor(self):
        self.joint_positions.data = [self.elbowpitch,self.wristpitch,-self.wristroll,self.gripangle,self.gripangle,-self.scissorangle,self.scissorangle,
                                self.scissorangle,-self.scissorangle,self.scissorangle,self.scissorangle,-self.gripangle,self.gripangle,self.gripangle,
                                -self.gripangle,self.gripangle,self.gripangle]
        self.joint_position_pub.publish(self.joint_positions)

    def calc(self):
        for i in range(0,steps):
            t = i/steps*timesp # for loop increments from 1 to the total endtime
            # Get the new most recent pose based on FORWARD KINEMATICS
            newpose = lambdifyTMD(self.q[0],self.q[1],self.q[2],self.q[3],
                                  self.q[4],self.q[5])
            # VELOCITY in x and z. y vel is 0. 
            xyvel = -5*math.sin(angV*t)*angV
            zvel = -15*math.sin(angV*t)*angV
            Xdot = Matrix([xyvel, xyvel, zvel, 0, 0, 0])
            # tempJacobian = JacobianMat.subs({theta0:q[0],theta1:q[1],theta2:q[2],
            #                   theta3:q[3],theta4:q[4],theta5:q[5]})
            tempJacobian = lambdifyJacob(self.q[0],self.q[1],self.q[2],self.q[3],
                                         self.q[4],self.q[5])
            lPseudoInv = (tempJacobian.T*tempJacobian)**-1*tempJacobian.T
            qdot = lPseudoInv*Xdot
            # qdot = tempJacobian**-1*Xdot
            self.q += qdot*timesp/steps
            self.qv.append(self.q) # generate path first

    # always ends in the "up" position so another method can drop the hand and change to the appropriate sign
    def shakefists(self):
        tperstep = timesp/steps
        for j in range(3):  # three bounces as per RCS rules                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
            for i in range(steps):
                # we only use the first three joints since the hand is balled up
                self.elbowpitch = float(self.qv[i][0]*self.scale)
                self.wristpitch = float(self.qv[i][1]*self.scale)
                self.wristroll = float(self.qv[i][2]*self.scale)
                self.pubjoints()
                time.sleep(tperstep)
            for i in range(steps):
                self.elbowpitch = float(self.qv[steps-i-1][0]*self.scale)
                self.wristpitch = float(self.qv[steps-i-1][1]*self.scale)
                self.wristroll = float(self.qv[steps-i-1][2]*self.scale)
                self.pubjoints()
                time.sleep(tperstep)

    def rock(self):
        self.shakefists()
        tperstep = timesp/steps                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
        for i in range(0,steps):
            # we only use the first three joints since the hand is balled up
            self.elbowpitch = float(self.qv[i][0]*self.scale)
            self.wristpitch = float(self.qv[i][1]*self.scale)
            self.wristroll = float(self.qv[i][2]*self.scale)
            self.pubjoints()
            time.sleep(tperstep)
        time.sleep(2)
        for i in range(steps):
            self.elbowpitch = float(self.qv[steps-i-1][0]*self.scale)
            self.wristpitch = float(self.qv[steps-i-1][1]*self.scale)
            self.wristroll = float(self.qv[steps-i-1][2]*self.scale)
            self.pubjoints()
            time.sleep(tperstep)

    def paper(self):
        self.shakefists()        
        tperstep = timesp/steps
        dangle = float(2*pi/5/timesp/10)
        for i in range(0,steps):
            # we only use the first three joints since the hand is balled up
            self.elbowpitch = float(self.qv[i][0]*self.scale)
            self.wristpitch = float(self.qv[i][1]*self.scale)
            self.wristroll = float(self.qv[i][2]*self.scale)
            if(i > steps/2):
                if(self.gripangle > 0):
                    self.gripangle -= dangle
            self.pubjoints()
            time.sleep(tperstep)
        time.sleep(2)
        for i in range(steps):
            self.elbowpitch = float(self.qv[steps-i-1][0]*self.scale)
            self.wristpitch = float(self.qv[steps-i-1][1]*self.scale)
            self.wristroll = float(self.qv[steps-i-1][2]*self.scale)
            if(i > steps/2):
                if(self.gripangle < 2/5*pi):
                    self.gripangle += dangle
            self.pubjoints()
            time.sleep(tperstep)

    def scissors(self):
        self.shakefists()
        tperstep = timesp/steps
        dangle = float(2*pi/5/timesp/10)
        for i in range(0,steps):
            # we only use the first three joints since the hand is balled up
            self.elbowpitch = float(self.qv[i][0]*self.scale)
            self.wristpitch = float(self.qv[i][1]*self.scale)
            self.wristroll = float(self.qv[i][2]*self.scale)
            if(i > steps/2):
                if(self.gripangle > 0):
                    self.scissorangle -= dangle
            self.pubjoints_scissor()
            time.sleep(tperstep)
        time.sleep(2)
        for i in range(steps):
            self.elbowpitch = float(self.qv[steps-i-1][0]*self.scale)
            self.wristpitch = float(self.qv[steps-i-1][1]*self.scale)
            self.wristroll = float(self.qv[steps-i-1][2]*self.scale)
            if(i > steps/2):
                if(self.gripangle < 2/5*pi):
                    self.scissorangle += dangle
            self.pubjoints_scissor()
            time.sleep(tperstep)

    def throw(self):
        rng = np.random.randint(3)
        print(rng)
        if(rng == 0):
            self.rock()
        elif(rng == 1):
            self.paper()
        else:
            self.scissors()

def main(args=None):
    rclpy.init(args=args)   
    node = RPS()
    node.calc()
    node.throw()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
