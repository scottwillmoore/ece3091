import numpy as np
import cv2
from matplotlib import pyplot as plt
import torch
import torch.nn as nn
import csv
import os

# https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html


# 233 also reasonable from first set
"""image = cv2.imread('ims1/im_0005.png')
plt.imshow(image)
plt.show()"""

# if you want accurate number fpr the image start count at 1 less than what you want
count = 0
for filepath in os.listdir('ims1/'):
    count = count + 1
    if count % 5 == 0:
        image = cv2.imread('ims1/{0}'.format(filepath),1)

        # grey scale image
        greyImage = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        greyImage = cv2.medianBlur(greyImage, 5)

        rows = greyImage.shape[0]

        # param 2 may need to increase for other images, DO NOT DECREASE BELOW 27, DO NOT INCREASE ABOVE 30 (exception would be using our data from NoIR)
        # param 1 DO NOT DECREASE BELOW 81
        circles = cv2.HoughCircles(greyImage, cv2.HOUGH_GRADIENT,1,rows/10,param1=100,param2=29,minRadius = 1,maxRadius=50)
        # the variable circles is shape (3, number of circles, 1)
        # the data of circles is x centre, y centre, radius
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                centre = (i[0],i[1])
                cv2.circle(image,centre,1,(0,100,100),3)
                radius = i[2]
                cv2.circle(image, centre,radius, (255,0,255),3)

            # make a new variable to store the circles in
            circlesArray = np.zeros_like(circles)
            for i in range(len(circles)):
                circlesArray[i] = circles[i]

            # show the circles using cv2
            """cv2.imshow("detected circles",image)
            cv2.waitKey()"""

            # show the images using matplotlib
            fig,ax = plt.subplots()
            for i in range(0,circles.shape[1]):
                circle = plt.Circle((circlesArray[0,i,0],circlesArray[0,i,1]),circlesArray[0,i,2],fill=False)
                ax.add_artist(circle)
            plt.imshow(image)
            plt.xlim(0,320) # scale of provided noir images
            plt.ylim(0,160)
            ax.set_ylim(ax.get_ylim()[::-1]) # inverts the y-axis to make it similar to the images
            plt.show()

            # the code above this works super well and is amazing
            # the code below here is untested, crystal plans to work on this idea

            diameter = circlesArray[0,0,2]
            squareSize = np.sqrt(diameter**2/2)
            squareHalf = int(squareSize/2)
            left = circlesArray[0,0,0]-squareHalf
            right = circlesArray[0,0,0]+squareHalf
            top = circlesArray[0,0,1]+squareHalf
            bottom = circlesArray[0,0,1]-squareHalf

            print(top,bottom,left,right)
            redVec = []
            blueVec = []
            greenVec = []
            hueVec = []
            satVec = []
            valueVec = []

            print(image[130,253,0])
            for i in range(bottom,top):
                for j in range(left,right):
                    red = image[i,j,0]
                    green = image[i,j,1]
                    blue = image[i,j,2]
                    redVec.append(red)
                    greenVec.append(green)
                    blueVec.append(blue)
                    """hue = hsv[i,j,0]
                    sat = hsv[i,j,1]
                    value = hsv[i,j,2]
                    hueVec.append(hue)
                    satVec.append(sat)
                    valueVec.append(value)"""

            print('Red',np.mean(redVec),np.std(redVec))
            print('Green',np.mean(greenVec),np.std(greenVec))
            print('Blue',np.mean(blueVec),np.std(blueVec))

        else:
            print("No circles identified in image",count)

# we tried Sobel and it doesn't work by itself very well





# edge detection code that produces edges
# can change the thresholds
blurImage = cv2.GaussianBlur(greyImage,(3,3),0)
edges = cv2.Canny(image=blurImage,threshold1=100,threshold2=200)
cv2.imshow('Edges',edges)
cv2.waitKey(0)

# find the contours from the edge detection
contours,hierarchy = cv2.findContours(edges,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
for c in contours:
    hull = cv2.convexHull(c)
    cv2.drawContours(image,[hull],0,(0,255,0),thickness=1)
fig, ax = plt.subplots()
plt.imshow(image)
plt.show()