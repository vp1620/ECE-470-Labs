#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm, logm
from lab5_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	
	M = np.array([[0, 1, 0, .39], [0, 0, 1, .401], [-1, 0, 0, .2155], [0, 0, 0, 1]])
	S1 = np.array([[0, -1, 0, .15], [1, 0, 0, .15], [0, 0, 0, 0], [0, 0, 0, 0]])
	S2 = np.array([[0, 0, 1, -.162], [0, 0, 0, 0], [-1, 0, 0, -.15], [0, 0, 0, 0]])
	S3 = np.array([[0, 0, 1, -.162], [0, 0, 0, 0], [-1, 0, 0, .094], [0, 0, 0, 0]])
	S4 = np.array([[0, 0, 1, -.162], [0, 0, 0, 0], [-1, 0, 0, .307], [0, 0, 0, 0]])
	S5 = np.array([[0, 0, 0, 0], [0, 0, -1, .163], [0, 1, 0, -.26], [0, 0, 0, 0]])
	S6 = np.array([[0, 0, 1, -.162], [0, 0, 0, 0], [-1, 0, 0, .390], [0, 0, 0, 0]])





	# ==============================================================#
	return M, S1, S2, S3, S4, S5, S6


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	#print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	values = Get_MS()
	e1 = expm(theta1*values[1])
	e2 = expm(theta2*values[2])
	e3 = expm(theta3*values[3])
	e4 = expm(theta4*values[4])
	e5 = expm(theta5*values[5])
	e6 = expm(theta6*values[6])

	T_theta = e1@e2@e3@e4@e5@e6@values[0]

	# ==============================================================#

	#print(str(T_theta) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6
	

	return return_value 


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripRad):
	# =================== Your code starts here ====================#

	xgrip  = xWgrip + .15
	ygrip = yWgrip -.15
	zgrip = zWgrip -.01
	
	

	xcen = xgrip - .0535*np.cos(yaw_WgripRad)
	ycen = ygrip - .0535*np.sin(yaw_WgripRad)
	zcen = zgrip
	tandist = ((ycen**2)+(xcen**2)-(.11**2))**.5

	theta1 = (np.arctan2(ycen, xcen) - np.arctan2(.11, tandist))
	theta5 = (-90*np.pi)/180
	theta6 = (theta5*-1) + theta1 - yaw_WgripRad
	


	x3end = xcen - .083*np.cos(theta1)+.11*np.sin(theta1)
	y3end = ycen - .083*np.sin(theta1)-.11*np.cos(theta1)
	z3end = zcen + .141
	
	dist1 = ((y3end**2)+(x3end**2))**.5
	psiVar = np.arctan2(z3end - .152, dist1)
	c = ((y3end**2)+(x3end**2)+((z3end-.152)**2))**.5
	a = .244
	b = .213
	
	angC = np.arccos(((-1*c**2) + a**2 + b**2)/(2*a*b))
	angB = np.arccos(((-1*b**2) + a**2 + c**2)/(2*a*c))
	# angA = np.arccos(((-1*a**2) + b**2 + c**2)/(2*b*c))

	
	theta2 = (psiVar + angB)*-1
	theta3 = np.radians(180 - ((angC*180)/np.pi))
	theta4 = -1*theta3 - theta2
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)