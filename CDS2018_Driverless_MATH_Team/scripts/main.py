#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import cv2
import numpy as np
import math
from sensor_msgs.msg import CompressedImage
import sys
import matplotlib.pyplot as plt
# our code import
import detectlane 

# out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (320,240))
CAR_POS_X = 120
CAR_POS_Y = 300
slideThickness = 10
VERTICAL = 0
HORIZONTAL = 1

class image:
    def __init__(self):
        self.speed_pub = rospy.Publisher("Team1_speed",Float32,queue_size=10)
        self.steerAngle_pub = rospy.Publisher("Team1_steerAngle",Float32,queue_size=10)
        self.subscriber = rospy.Subscriber("Team1_image/compressed",CompressedImage, self.callback,  queue_size = 1)

    def callback(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # CODE START FROM HERE
        # lane = detectlane.Lane()
        # lane.imgProcess(image_np)

        binary = toBinary(image_np)
        smooth = smoothImage(binary)
        hsv = toHsv(image_np)
        bird = birdViewTransform(hsv)
        canny = cannyEdge(hsv)
        cv2.imshow('img',hsv)
        
        
        layers = splitLayer(bird, VERTICAL)
        points = centerRoadSide(layers)

        for i in range(len(points)):
            for j in range(len(points[i])):
                cv2.circle(bird, points[i][j],2,(255,255,255),thickness=1, lineType=8)
        cv2.imshow('point',bird) 
        binary = toBinary(image_np)
        smooth = smoothImage(binary)
        hsv = toHsv(image_np)
        bird = birdViewTransform(hsv)
        canny = cannyEdge(hsv)
        cv2.imshow('img',hsv)
        
        
        layers = splitLayer(bird, VERTICAL)
        points = centerRoadSide(layers)

        for i in range(len(points)):
            for j in range(len(points[i])):
                cv2.circle(bird, points[i][j],2,(255,255,255),thickness=1, lineType=8)
        cv2.imshow('point',bird)
        cv2.waitKey(2)
        

# Convert IMG to binary
def toBinary(image):
    binary = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    thresh, res = cv2.threshold(binary, 160, 255, cv2.THRESH_BINARY)
    return res

# Convert IMG to HSV
def toHsv(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    res = cv2.inRange(hsv, (0,0,180),(179,30,255))
    dst = cv2.dilate(res, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)))
    return dst

# def getAngle(nextPoint):
#     dstX = nextPoint[0]
#     dstY = nextPoint[1]
#     if(dstX == CAR_POS_X):
#         return 0
#     if(dstY == CAR_POS_Y):
#         return (dstX < CAR_POS_X ? -90 : 90)
#     pi = acos(-1)
#     dx = dstX - CAR_POS_X
#     dy = dstY - CAR_POS_Y
#     if(dx < 0):
#         return -atan(-dx/dy)*180/pi
#     return atan(dx/dy)*180/pi

def splitLayer(img, direction):
    height, width = img.shape
    res = []
    if(direction == VERTICAL):
        for i in range(0,height-slideThickness,slideThickness):
            layer = img[i:i+slideThickness-1, :]
            res.append(layer)
    else:
        for i in range(0,width-slideThickness,slideThickness):
            layer = img[height-1, i:width+slideThickness-1]
            res.append(layer)
    return res

def centerRoadSide(listLayers):
    res = []
    for i in range(len(listLayers)):
        _,cnts,__ = cv2.findContours(listLayers[i], cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cntSize = len(cnts)
        tmp=[]
        if(cntSize == 0): 
            res.append(tmp)
            continue
        for j in range(0,cntSize):
            if(cv2.contourArea(cnts[j],False) > 3):
                # calculate moments of binary image
                M = cv2.moments(cnts[j])
                
                # calculate x,y coordinate of center
                cX = int(M["m10"] / M["m00"])

                cY = int(M["m01"] / M["m00"]) + slideThickness*i
                print (cX,cY)
                tmp.append((cX,cY))
        res.append(tmp)
    return res
# vector<vector<Point> > DetectLane::centerRoadSide(const vector<Mat> &src, int dir)
# {
#     vector<std::vector<Point> > res;
#     int inputN = src.size();
#     for (int i = 0; i < inputN; i++) {
#         std::vector<std::vector<Point> > cnts;
#         std::vector<Point> tmp;
#         findContours(src[i], cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
#         int cntsN = cnts.size();
#         if (cntsN == 0) {
#             res.push_back(tmp);
#             continue;
#         }
#         for (int j = 0; j < cntsN; j++) {
#             int area = contourArea(cnts[j], false);
#             if (area > 3) {
#                 Moments M1 = moments(cnts[j], false);
#                 Point2f center1 = Point2f(static_cast<float> (M1.m10 / M1.m00), static_cast<float> (M1.m01 / M1.m00));
#                 if (dir == VERTICAL) {
#                     center1.y = center1.y + slideThickness*i;
#                 } 
#                 else
#                 {
#                     center1.x = center1.x + slideThickness*i;
#                 }
#                 if (center1.x > 0 && center1.y > 0) {
#                     tmp.push_back(center1);
#                 }
#             }
#         }
#         res.push_back(tmp);
#     }

#     return res;
# }


def smoothImage(image):
    res = cv2.blur(image, (2, 2),0)
    return res

def cannyEdge(image):
    res = cv2.Canny(image, 50,100,3)
    return res

def birdViewTransform(image):
    height, width = image.shape
    src_vertices = np.zeros((4, 2), dtype = "float32")
    src_vertices[0] = np.asarray((0,85))
    src_vertices[1] = np.asarray((width,85))
    src_vertices[2] = np.asarray((width, height))
    src_vertices[3] = np.asarray((0, height))

    dst_vertices = np.zeros((4, 2), dtype = "float32")
    dst_vertices[0] = np.asarray((0,0))
    dst_vertices[1] = np.asarray((240,0))
    dst_vertices[2] = np.asarray((240-105, 320))
    dst_vertices[3] = np.asarray((105, 320))

    M = cv2.getPerspectiveTransform(src_vertices, dst_vertices)
    warped = cv2.warpPerspective(image, M, (240, 320))
    # drawHistogram(warped)
    # findContours(warped)
    cv2.imshow('birdview', warped)
    cv2.waitKey(2)

    return warped

def findContours(img):
    _,cnts,_ = cv2.findContours(img, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cntSize = len(cnts)
    for i in range(0,cntSize):
        if(cv2.contourArea(cnts[i]) > 20):
            cv2.drawContours(img, cnts[i], 1, (0,0,255), 1)
    return

def drawHistogram(image):
    histogram = np.sum(image[image.shape[0]//2:,:], axis=0)
    plt.plot(histogram)
    plt.show()
    cv2.waitKey(30)
    return

def main(args):
    img = image()
    rospy.init_node('image_reciver', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
