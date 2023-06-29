#!/usr/bin/env python3

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([148.77, -85.70, 126.36, -134.90, -88.50, 62.81])

# Hanoi tower location 1
Q11 = [136.97*pi/180.0, -78.29*pi/180.0, 116.62*pi/180.0, -127.14*pi/180.0, -90.23*pi/180.0, 66.42*pi/180.0]
Q12 = [136.95*pi/180.0, -73.33*pi/180.0, 119.80*pi/180.0, -135.28*pi/180.0, -90.26*pi/180.0, 66.46*pi/180.0]
Q13 = [139.23*pi/180.0, -66.24*pi/180.0, 118.51*pi/180.0, -136.06*pi/180.0, -91.42*pi/180.0, 77.75*pi/180.0]
Q14 = [136.91*pi/180.0, -58.36*pi/180.0, 123.68*pi/180.0, -154.13*pi/180.0, -90.33*pi/180.0, 66.52*pi/180.0]

# Hanoi tower location 2
Q21 = [148.77*pi/180.0, -85.70*pi/180.0, 126.36*pi/180.0, -134.90*pi/180.0, -88.50*pi/180.0, 62.81*pi/180.0]
Q22 = [148.77*pi/180.0, -77.51*pi/180.0, 130.36*pi/180.0, -144.90*pi/180.0, -87.50*pi/180.0, 63.81*pi/180.0]
Q23 = [148.75*pi/180.0, -68.49*pi/180.0, 132.81*pi/180.0, -155.90*pi/180.0, -88.60*pi/180.0, 62.9*pi/180.0]
Q24 = [149.47*pi/180.0, -58.68*pi/180.0, 132.13*pi/180.0, -165.99*pi/180.0, -89.03*pi/180.0, 62.89*pi/180.0]

# Hanoi tower location 3
Q31 = [172.69*pi/180.0, -80.39*pi/180.0, 119.06*pi/180.0, -128.66*pi/180.0, -89.41*pi/180.0, 26.10*pi/180.0]
Q32 = [171.74*pi/180.0, -71.77*pi/180.0, 121.78*pi/180.0, -142.17*pi/180.0, -87.68*pi/180.0, 9.83*pi/180.0]
Q33 = [171.73*pi/180.0, -64.52*pi/180.0, 123.93*pi/180.0, -151.58*pi/180.0, -87.72*pi/180.0, 9.86*pi/180.0]
Q34 = [171.73*pi/180.0, -56.10*pi/180.0, 124.79*pi/180.0, -160.86*pi/180.0, -87.75*pi/180.0, 9.88*pi/180.0]

# Q23 means tower 2nd, 3rd point from top of tower 2


thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13, Q14], \
      [Q21, Q22, Q23, Q24], \
      [Q31, Q32, Q33, Q34]]


############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def gripper_callback(msg):
    
   global analog_in_0

   analog_in_0 = msg.AIN0

    

    



############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q
    global analog_in_0
    ### Hint: Use the Q array to map out your towers by location and "height".

    ## move_arm(pub_cmd, loop_rate, dest, vel, accel)
    ## gripper(pub_cmd, loop_rate, io_0)

    

    start_arm = Q[start_loc- 1][0]
    sArmHeight = Q[start_loc - 1][start_height - 1]
    end_arm = Q[end_loc- 1][0]
    eArmHeight = Q[end_loc - 1][end_height - 1]

    move_arm(pub_cmd, loop_rate, start_arm, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, sArmHeight, 4.0, 4.0)
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(1.0)
    if (analog_in_0 < 1.9):

        gripper(pub_cmd, loop_rate, suction_off)
        print("Block not found... ")
        sys.exit()

    move_arm(pub_cmd, loop_rate, start_arm, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, end_arm, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, eArmHeight, 4.0, 4.0)
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(1.0)
    move_arm(pub_cmd, loop_rate, end_arm, 4.0, 4.0)

    error = 0

    return error

############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function

    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)


    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    start_done = 0
    end_done = 0
    startPos= 0
    endPos = 0
    

    while(not start_done):
        input_string = input("Enter start position anywhere from 0-3 with 0 being quit 1-3 being positions of towers:")
        print("You entered " + input_string + "\n")

        if(int(input_string) == 1):

            startPos = 1
            start_done = 1

        elif (int(input_string) == 2):

            startPos = 2
            start_done = 1

        elif (int(input_string) == 3):

            startPos = 3
            start_done = 1

        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")
            

    
    while(not end_done):
        input_string = input("Enter end position anywhere from 0-3 with 0 being quit 1-3 being positions of towers:")
        print("You entered " + input_string + "\n")

        if(int(input_string) == 1):

            endPos = 1
            end_done = 1

        elif (int(input_string) == 2):

            endPos = 2
            end_done = 1

        elif (int(input_string) == 3):

            endPos = 3
            end_done = 1

        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")





    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    #while(loop_count > 0):

     #   move_arm(pub_command, loop_rate, home, 4.0, 4.0)

      #  rospy.loginfo("Sending goal ...")
       # move_arm(pub_command, loop_rate, Q[startPos - 1][0], 4.0, 4.0)

        #gripper(pub_command, loop_rate, suction_on)
        # Delay to make sure suction cup has grasped the block
        #time.sleep(1.0)
    midPos = 6 - (startPos + endPos)
    move_block(pub_command, loop_rate, startPos, 2, endPos, 4)
    move_block(pub_command, loop_rate, startPos, 3, midPos, 4)
    move_block(pub_command, loop_rate, endPos, 4, midPos, 3)
    move_block(pub_command, loop_rate, startPos, 4, endPos, 4)
    move_block(pub_command, loop_rate, midPos, 3, startPos, 4)
    move_block(pub_command, loop_rate, midPos, 4, endPos, 3)
    move_block(pub_command, loop_rate, startPos, 4, endPos, 2)

    gripper(pub_command, loop_rate, suction_off)



    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
