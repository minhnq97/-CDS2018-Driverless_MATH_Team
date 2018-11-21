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
        # substractBackground(image_np)
        cv2.imshow('image',image_np)
        cv2.waitKey(2)
        binary = toBinary(image_np)
        hsv = toHsv(image_np)
        smooth = smoothImage(hsv)
        bird = birdViewTransform(hsv)
        canny = cannyEdge(hsv)
        # houghTransform(canny,image_np)
        cv2.imshow('img',smooth)
        cv2.waitKey(2)
        layers = splitLayer(bird, VERTICAL)
        points = centerRoadSide(layers)
        leftLane, rightLane = detectLeftRight(points)
        birdPoints = np.zeros((320,240,3), np.uint8)
        for i in range(len(leftLane)):
            if (leftLane[i] != None):
                cv2.circle(birdPoints, leftLane[i],2,(255,0,0),thickness=1, lineType=8)

        for i in range(len(rightLane)):
            if (rightLane[i] != None):
                cv2.circle(birdPoints, rightLane[i],2,(0,0,255),thickness=1, lineType=8)

        e,v = carControl(leftLane, rightLane)
        self.speed_pub.publish(v)
        self.steerAngle_pub.publish(e)
        cv2.imshow('point',birdPoints)
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

def carControl(leftLane, rightLane):
    index = len(leftLane) - 11
    error = 0
    while (leftLane[index] == None and rightLane[index] == None):
        index-=1
        if (index < 0):
            return
    
    if (leftLane[index] != None and rightLane[index] != None):
        error = getAngle(((leftLane[index][0] + rightLane[index][0]) / 2, (leftLane[index][1] + rightLane[index][1]) / 2))
    elif (leftLane[index] != None):
        error = getAngle((leftLane[index][0]+20,leftLane[index][1]))
    else:
        error = getAngle((rightLane[index][0]-20,rightLane[index][1]))

    velocity = 30
    return error, velocity

def getAngle(nextPoint):
    dstX = float(nextPoint[0])
    dstY = float(nextPoint[1])
    # print 'point: {},{}'.format(nextPoint[0],nextPoint[1])

    if(dstX == CAR_POS_X):
        return 0
    if(dstY == CAR_POS_Y):
        return (dstX < CAR_POS_X if -90 else 90)
    pi = math.acos(-1)
    dx = dstX - CAR_POS_X
    dy = dstY - CAR_POS_Y
    dy=-dy
    if(dx < 0):
        print 'error: {}; dx: {}; dy: {}'.format((math.atan(-dx/dy)*180/pi),dx,dy)
        return (-math.atan(-dx/dy)*180/pi)
    print 'error: {}; dx: {}; dy: {}'.format((math.atan(dx/dy)*180/pi),dx,dy)
    return math.atan(dx/dy)*180/pi

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
                tmp.append((cX,cY))
        res.append(tmp)
    return res

def detectLeftRight(points):
    lane1 = []
    lane2 = []
    leftLane = []
    rightLane = []
    for i in range(320/slideThickness):
        leftLane.append(None)
        rightLane.append(None)

    pointMap = [[0 for i in range(20)] for j in range(len(points))]
    prePoint = [[0 for i in range(20)] for j in range(len(points))]
    postPoint = [[0 for i in range(20)] for j in range(len(points))]
    dis = 10
    max = -1
    max2 = -1
    posMax = ()
    posMax2 = ()

    for i in range(len(points)):
        for j in range(len(points[i])):
            pointMap[i][j] = 1
            prePoint[i][j] = -1
            postPoint[i][j] = -1

    for i in range(len(points)-2, -1, -1):
        for j in range(len(points[i])):
            err = 320
            for m in range(1,min(len(points)-1-i,5)):
                check = False
                for k in range(len(points[i+1])):
                    a = points[i+m][k][0]
                    # print '{} and {} and {}'.format(i,m,k)
                    b = points[i][j][0]
                    if(abs(a - b) < dis
                    and abs(points[i + m][k][0] - points[i][j][0]) < err):
                        err = abs(points[i + m][k][0] - points[i][j][0])
                        pointMap[i][j] = pointMap[i + m][k] + 1
                        prePoint[i][j] = k
                        postPoint[i + m][k] = j
                        check = True
                break
            if(pointMap[i][j] > max):
                max = pointMap[i][j]
                posMax = (i,j)

    for i in range(len(points)):
        for j in range(len(points[i])):
            if(pointMap[i][j] > max2
            and (i != posMax[0] or j != posMax[1])
            and postPoint[i][j] == -1):
                max2 = pointMap[i][j]
                posMax2 = (i,j)

    if(max==-1):
        return
    
    while (max>=1):
        lane1.append(points[posMax[0]][posMax[1]])
        if(max==1):
            break
        posMax = (posMax[0]+1, prePoint[posMax[0]][posMax[1]])
        # posMax[1] = prePoint[posMax[0]][posMax[1]]
        # posMax[0] += 1
        max -= 1

    while(max2>=1):
        lane2.append(points[posMax2[0]][posMax2[1]])
        if(max2==1):
            break
        posMax2 = (posMax2[0]+1, prePoint[posMax2[0]][posMax2[1]])
        # posMax2[1] = prePoint[posMax2[0]][posMax2[1]]
        # posMax2[0] += 1
        max2 -= 1

    subLane1 = lane1[0:5]
    subLane2 = lane2[0:5]
    vx1, vy1, x1, y1 = cv2.fitLine(np.array(subLane1), 2, 0, 0.01, 0.01)
    vx2, vy2, x2, y2 = cv2.fitLine(np.array(subLane2), 2, 0, 0.01, 0.01)
    line1 = [vx1, vy1, x1, y1]
    line2 = [vx2, vy2, x2, y2]
    lane1X = (240 - line1[3]) * line1[0] / line1[1] + line1[2]
    lane2X = (240 - line2[3]) * line2[0] / line2[1] + line2[2]

    if(lane1X < lane2X):
        # print 'lane1x < lane2x'
        for i in range(len(lane1)):
            leftLane[int(math.floor(lane1[i][1] / slideThickness))] = lane1[i]

        for i in range(len(lane2)):
            rightLane[int(math.floor(lane2[i][1] / slideThickness))] = lane2[i]
    else:
        # print 'lane1x > lane2x'
        for i in range(len(lane2)):
            leftLane[int(math.floor(lane2[i][1] / slideThickness))] = lane2[i]

        for i in range(len(lane1)):
            rightLane[int(math.floor(lane1[i][1] / slideThickness))] = lane1[i]    

    return leftLane, rightLane


def smoothImage(image):
    res = cv2.blur(image, (2, 2),0)
    return res

def cannyEdge(image):
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(image,(kernel_size, kernel_size),0)
    res = cv2.Canny(blur_gray, 50,150)
    return res

def substractBackground(img):
    rgb_planes = cv2.split(img)
    result_planes = []
    result_norm_planes = []
    for plane in rgb_planes:
        dilated_img = cv2.dilate(plane, np.ones((7,7), np.uint8))
        bg_img = cv2.medianBlur(dilated_img, 21)
        diff_img = 255 - cv2.absdiff(plane, bg_img)
        norm_img = diff_img.copy()
        cv2.normalize(diff_img,norm_img, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
        result_planes.append(diff_img)
        result_norm_planes.append(norm_img)

    result = cv2.merge(result_planes)
    result_norm = cv2.merge(result_norm_planes)
    cv2.imshow('nonnorm',result)
    cv2.waitKey(2)
    return

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

def houghTransform(edges, img):
    cv2.imshow('canny',edges)
    lines = cv2.HoughLinesP(edges,1,np.pi/90,40, 200, 30)
    print len(lines)
    for i in range(len(lines)):
        for x1,y1,x2,y2 in lines[i]:
            if(x1!=x2 and y1!=y2):
                cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
    cv2.imshow('hough', img)
    return

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
