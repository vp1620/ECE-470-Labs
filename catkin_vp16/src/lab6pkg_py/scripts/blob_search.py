#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = np.arctan(1/38)
beta = 745.0
tx = 218/beta
ty = 42/beta

# Function that converts image coord to world coord
def IMG2W(col, row):

    xc = (row-240)/beta
    yc = (col-320)/beta
    # print(row)
    # print(col)
    
    xyCam = np.array([[xc + tx], [yc + ty]])
    Rot  = np.array([[np.cos(theta), -1*np.sin(theta)], [np.sin(theta), np.cos(theta)]])

    xyWorld = Rot@xyCam

    return xyWorld

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 200
    params.maxArea = 700

    # Filter by Circularity
    params.filterByCircularity = False
    params.maxCircularity = 0.2

    # Filter by Inerita
    params.filterByInertia = True
    params.minInertiaRatio = 0.6

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================


    # lowerR = (90, 50, 50)     # green lower
    # upperR = (127,255,255)   # green upper
    
    # lowerG = (128,100,100)     # middle lower
    # upperG = (145,255,255)   # middle upper

    # lowerY = (0,100,100)     # yellow lower
    # upperY = (15,255,255)   # yellow upper

    ####################################################################################

    lowerB = (110, 50, 50)     # blue lower
    upperB = (135,255,255)   # blue upper
    
    lowerR = (0,100,100)     # red lower
    upperR = (20,255,255)   # red upper

    lowerG = (50,50,50)     # green lower
    upperG = (90,255,255)   # green upper

    # Define a mask using the lower and upper bounds of the target color

    if color == "red":

        mask_image = cv2.inRange(hsv_image, lowerR, upperR)
    
    elif color == "blue":
        
        mask_image = cv2.inRange(hsv_image, lowerB, upperB)

    elif color == "green":
        
        mask_image = cv2.inRange(hsv_image, lowerG, upperG)

    # ========================= Student's code ends here ==========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    #print(keypoints)
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, 0, (0, 0, 255), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)
    # print(xw_yw)

    return xw_yw
