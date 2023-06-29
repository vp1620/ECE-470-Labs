#!/usr/bin/env python

import sys
import copy
import time
import rospy

import numpy as np
from lab5_header import *
from lab5_func import *
from blob_search import *


# ========================= Student's code starts here =========================

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

# Store world coordinates of green, blue, and blue blocks

xw_yw_G = []
xw_yw_B = []
xw_yw_R = []

# Any other global variable you want to define
# Hints: where to put the blocks?


# ========================= Student's code ends here ===========================

################ Pre-defined parameters and functions no need to change below ################

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

image_shape_define = False


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):
    # global digital_in_0
    # digital_in_0 = msg.DIGIN
    # digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0

    global analog_in_0

    analog_in_0 = msg.AIN0


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


"""
Function to control the suction cup on/off
"""
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

            #rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""
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
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

################ Pre-defined parameters and functions no need to change above ################



def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, yawAngle, vel, accel):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates
    target_xw_yw_zw: where to place the block in global coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    global analog_in_0
    print(start_xw_yw_zw)
    print(yawAngle)

    sArmHeight= lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], .025, yawAngle[0])
    start_arm = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], .2, 0)
    end_arm = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], .2, 0)
    eArmHeight = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2], 0)
    intPos = lab_invk(.2, .2, .20, 0)



    if yawAngle < -PI/2 and yawAngle > -PI:

        print("-pi")
        newYaw = yawAngle[0] + PI

        twist_height = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], .025, newYaw)
        twist_arm= lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], .2, newYaw)
        act_twist = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], .025, newYaw + PI)
        act_height = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], .2, newYaw + PI)
        move_arm(pub_cmd, loop_rate, twist_arm, 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, twist_height, 4.0, 2.0)
        gripper(pub_cmd, loop_rate, suction_on)
        time.sleep(1.0)
        move_arm(pub_cmd, loop_rate, twist_arm, 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, act_height, 4.0, 4.0)
        time.sleep(0.5)
        move_arm(pub_cmd, loop_rate, act_twist, 4.0, 2.0)
        gripper(pub_cmd, loop_rate, suction_off)
        time.sleep(0.5)
        move_arm(pub_cmd, loop_rate, act_height, 4.0, 2.0)
        move_arm(pub_cmd, loop_rate, intPos, 4.0, 4.0)
        yawAngle[0] = newYaw
        


    elif yawAngle > PI/2 and yawAngle < PI:  ##still needs work
        
        print("pi")
        newYaw = yawAngle[0] - PI
        
        twist_height = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], .025, newYaw)
        twist_arm= lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], .2, newYaw)
        act_height = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], .2, newYaw + PI)
        act_twist = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], .025, newYaw + PI)
        move_arm(pub_cmd, loop_rate, twist_arm, 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, twist_height, 4.0, 2.0)
        gripper(pub_cmd, loop_rate, suction_on)
        time.sleep(1.0)
        move_arm(pub_cmd, loop_rate, twist_arm, 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, act_height, 4.0, 4.0)
        time.sleep(0.5)
        move_arm(pub_cmd, loop_rate, act_twist, 4.0, 2.0)
        gripper(pub_cmd, loop_rate, suction_off)
        time.sleep(0.5)
        move_arm(pub_cmd, loop_rate, act_height, 4.0, 2.0)
        move_arm(pub_cmd, loop_rate, intPos, 4.0, 4.0)
        yawAngle[0] = newYaw




    sArmHeight= lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], .025, yawAngle[0])
    start_arm = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], .2, 0)
    end_arm = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], .2, 0)
    eArmHeight = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2], 0)
    intPos = lab_invk(.2, .2, .20, 0)


    

    

    

    # print(sArmHeight, start_arm)


    move_arm(pub_cmd, loop_rate, start_arm, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, sArmHeight, 4.0, 2.0)
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(1.0)
    move_arm(pub_cmd, loop_rate, start_arm, 4.0, 4.0)
    

    # move_arm(pub_cmd, loop_rate, start_arm, 4.0, 2.0)
    # move_arm(pub_cmd, loop_rate, sArmHeight, 4.0, 2.0)
    # gripper(pub_cmd, loop_rate, suction_on)
    # time.sleep(1.0)


    # move_arm(pub_cmd, loop_rate, start_arm, 4.0, 2.0)
    move_arm(pub_cmd, loop_rate, end_arm, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, eArmHeight, 4.0, 2.0)
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(1.0)
    move_arm(pub_cmd, loop_rate, end_arm, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, intPos, 4.0, 4.0)

    error = 0

    return error


class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
            print("ROS is shutdown!")


    def image_callback(self, data):

        global xw_yw_G # store found green blocks in this list
        global xw_yw_B # store found blue blocks in this list
        global xw_yw_R # store found red blocks in this list

        try:
          # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.flip(raw_image, -1)
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        # You will need to call blob_search() function to find centers of green blocks
        # and blue blocks, and store the centers in xw_yw_G & xw_yw_B respectively.

        # If no blocks are found for a particular color, you can return an empty list,
        # to xw_yw_G or xw_yw_B.

        # Remember, xw_yw_G & xw_yw_B are in global coordinates, which means you will
        # do coordinate transformation in the blob_search() function, namely, from
        # the image frame to the global world frame.

        xw_yw_G = blob_search(cv_image, "green")
        xw_yw_B = blob_search(cv_image, "blue")
        xw_yw_R = blob_search(cv_image, "red")


"""
Program run from here
"""
def main():

    global go_away
    global xw_yw_B
    #global xw_yw_G
    global xw_yw_R

    # global variable1
    # global variable2

    # Initialize ROS node
    rospy.init_node('lab5node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, go_away, vel, accel)

    ic = ImageConverter(SPIN_RATE)
    time.sleep(5)

    # ========================= Student's code starts here =========================

    """
    Hints: use the found xw_yw_G, xw_yw_B to move the blocks correspondingly. You will
    need to call move_block(pub_command, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel)
    """
    # print(xw_yw_G)



    blueCoord = xw_yw_B
    redCoord = xw_yw_R
    greenCoord = xw_yw_G
    
    validR = []
    validY = []
    yawAngle = []
    approxCent = []
    validCenter = []
    # print(blueCoord)
    # print(redCoord)

    for r in range(len(redCoord)):
        for y in range(len(blueCoord)):

            dist = ((redCoord[r][0] - blueCoord[y][0])**2 + (redCoord[r][1] - blueCoord[y][1])**2)**0.5
            
            if (dist < .055 and dist > .045):

                validR.append(redCoord[r])
                validY.append(blueCoord[y])
                approxCent.append([(redCoord[r][0] + blueCoord[y][0])/2, (redCoord[r][1] + blueCoord[y][1])/2])

    for g in range(len(greenCoord)):

        for i in range(len(validR)):

        
            matchDist = ((greenCoord[g][0] - approxCent[i][0])**2 + (greenCoord[g][1] - approxCent[i][1])**2)**0.5


            if matchDist < .01:
            
                yawAngle.append(np.arctan2((validR[i][1]-validY[i][1]), (validR[i][0]-validY[i][0])))
                validCenter.append(greenCoord[g])
                    

            
    # print(greenCoord)    
    # print(approxCent)
    



    target_center  = [[.26, -.13, .025],
                    [.12, -.13, .025],
                    [.253, -.13, .05],
                    [.127, -.13, .05],
                    [.248, -.13, .075],
                    [.132, -.13, .075],
                    [.243, -.13, .100],
                    [.137, -.13, .100],
                    [.231, -.13, .125],
                    [.149, -.13, .125],
                    [.19, -.13, .15]]
    
    print(validCenter)
    print(greenCoord)
    print(validR)
    print(validY)

    for t in range(len(validCenter)):

        print(yawAngle[t])
        move_block(pub_command, loop_rate, validCenter[t], target_center[t], yawAngle[t], vel, accel)
        
        



    # ========================= Student's code ends here ===========================

    move_arm(pub_command, loop_rate, go_away, vel, accel)
    rospy.loginfo("Task Completed!")
    print("Use Ctrl+C to exit program")
    rospy.spin()

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass