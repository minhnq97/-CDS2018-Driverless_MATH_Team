#!/usr/bin/env python
import cv2
import numpy as np
import math

import detectshadow
import constant as const
import lanefilter
import curve

class Lane:
    slideThickness = 10
    
    def __init__(self):
        self.leftLane = []
        self.rightLane = []

    def getLeftLane(self):
        return self.leftLane

    def getRightLane(self):
        return self.rightLane

    def imgProcess(self, src):
        image_np = self.preProcess(src)
        # --- test1 -- Thanghv
        '''greyImg = self.toGrayImg(img)
        smoothImg = self.smoothImg(greyImg)
        edgeDetec = self.cannyEdgeDetection(smoothImg)
        edgeDetec = cv2.dilate(edgeDetec, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)))
        imgfill = self.fillPoly(smoothImg) '''
        # --------------------------------------------------------------4
        # --- test 2 ---- Minhnq 
        '''hsvImg = self.toHsvImg(image_np)
        smoothImg = self.smoothImg(hsvImg)
        canny = self.cannyEdgeDetection(smoothImg)
        houghTransform(canny,image_np)'''
        # --- test 3 ---- Minhnq
        p = { 'sat_thresh': 120, 'light_thresh': 40, 'light_thresh_agr': 205,
            'grad_thresh': (0.7, 1.4), 'mag_thresh': 40, 'x_thresh': 20 }        
        filtered = lanefilter.LaneFilter(p)
        processedImage = filtered.apply(image_np)
        birdView = self.birdViewTransform(processedImage)

        # hsvImg = self.toHsvImg(image_np)
        
        cv2.imshow('bird', processedImage)
        # Sliding window technique
        # curves = curve.Curves(number_of_windows = 10, margin = 5, minimum_pixels = 20)
        # res = curves.fit(birdView[:,:])
        # self.leftLane, self.rightLane = res['leftlane'], res['rightlane']
        # End sliding window technique

        # Collect point traditionally
        layers = self.splitLayer(birdView, const.LANE_VERTICAL)
        points = self.centerRoadSide(layers)
        self.leftLane, self.rightLane = self.detectLeftRight(points)
        birdPoints = np.zeros((320,240,3), np.uint8)
        for i in range(len(self.leftLane)):
            if (self.leftLane[i] != None):
                cv2.circle(birdPoints, self.leftLane[i],2,(255,0,0),thickness=1, lineType=8)

        for i in range(len(self.rightLane)):
            if (self.rightLane[i] != None):
                cv2.circle(birdPoints, self.rightLane[i],2,(0,0,255),thickness=1, lineType=8)
        # End collect point

        #show
        cv2.imshow('img',image_np)
        cv2.imshow('point',birdPoints)

    def preProcess(self, img):
        # imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # imgThresholded = cv2.inRange(imgHSV, self.minThreshold, self.maxThreshold)
        # imgHSV = cv2.bitwise_and(img, img, mask = imgThresholded )
        return img

    # Convert IMG to BinaryIMG
    def toBinaryImg(self, img):
        thresh = 160    # threshold value
        maxval = 255    # maximum value
        binaryImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        thresh, res = cv2.threshold(binaryImg, thresh, maxval, cv2.THRESH_BINARY)
        return res

    # Convert IMG to GrayIMG
    def toGrayImg(self, img):
        grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return grayImg

    # Convert IMG to HSVIMG
    def toHsvImg(self, img):
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        res = cv2.inRange(hsvImg, const.LANE_MINTHRESHOLD, const.LANE_MAXTHRESHOLD)

        kernel_size = 5
        dst = cv2.dilate(res, 
                    cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)))
        return dst

    # Smooth IMG    
    def smoothImg(self, img):
        kernel_size = 5
        res = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
        return res

    # Convert IMG to BirdView
    def birdViewTransform(self, img):
        height, width = img.shape
        src_vertices = np.zeros((4, 2), dtype = "float32")
        src_vertices[0] = np.asarray((0, const.LANE_SKYLINE))
        src_vertices[1] = np.asarray((width, const.LANE_SKYLINE))
        src_vertices[2] = np.asarray((width, height))
        src_vertices[3] = np.asarray((0, height))

        dst_vertices = np.zeros((4, 2), dtype = "float32")
        dst_vertices[0] = np.asarray((0,0))
        dst_vertices[1] = np.asarray((const.LANE_BIRDVIEW_WIDTH, 0))
        dst_vertices[2] = np.asarray((const.LANE_BIRDVIEW_WIDTH - 105, const.LANE_BIRDVIEW_HEIGHT))
        dst_vertices[3] = np.asarray((105, const.LANE_BIRDVIEW_HEIGHT))

        # Test new birdView
        # src_vertices[0] = np.asarray((110, const.LANE_SKYLINE))
        # src_vertices[1] = np.asarray((240, const.LANE_SKYLINE))
        # src_vertices[2] = np.asarray((width, height))
        # src_vertices[3] = np.asarray((0, height))

        # dst_vertices = np.zeros((4, 2), dtype = "float32")
        # dst_vertices[0] = np.asarray((40,0))
        # dst_vertices[1] = np.asarray((280, 0))
        # dst_vertices[2] = np.asarray((280, 240))
        # dst_vertices[3] = np.asarray((40, 240))
        # # End test new birdView

        M = cv2.getPerspectiveTransform(src_vertices, dst_vertices)
        warped = cv2.warpPerspective(img, M, (const.LANE_BIRDVIEW_WIDTH, const.LANE_BIRDVIEW_HEIGHT))
        return warped

    # Detection canny Edge
    def cannyEdgeDetection(self, img):
        low_threshold = 50
        high_threshold = 50
        edges = cv2.Canny(img, low_threshold, high_threshold)
        return edges

    # Hough line 
    def houghTransform(self, edges, img):
        lines = cv2.HoughLinesP(edges,1,np.pi/90,40, 200, 30)
        for i in range(len(lines)):
            for x1,y1,x2,y2 in lines[i]:
                if(x1!=x2 and y1!=y2):
                    cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
        return

    # Split layer 
    def splitLayer(self, img, direction):
        height, width = img.shape
        res = []
        if(direction == const.LANE_VERTICAL):
            for i in range(0,height-self.slideThickness,self.slideThickness):
                layer = img[i:i+self.slideThickness-1, :]
                res.append(layer)
        else:
            for i in range(0,width-self.slideThickness,self.slideThickness):
                layer = img[height-1, i:width+self.slideThickness-1]
                res.append(layer)
        return res
    
    # Get point center
    def centerRoadSide(self, listLayers):
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

                    cY = int(M["m01"] / M["m00"]) + self.slideThickness*i
                    # print (cX,cY)
                    tmp.append((cX,cY))
            res.append(tmp)
        return res

    # Get point left - right
    def detectLeftRight(self, points):
        lane1 = []
        lane2 = []
        leftLane = []
        rightLane = []
        for i in range(320/self.slideThickness):
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
                leftLane[int(math.floor(lane1[i][1] / self.slideThickness))] = lane1[i]

            for i in range(len(lane2)):
                rightLane[int(math.floor(lane2[i][1] / self.slideThickness))] = lane2[i]
        else:
            # print 'lane1x > lane2x'
            for i in range(len(lane2)):
                leftLane[int(math.floor(lane2[i][1] / self.slideThickness))] = lane2[i]

            for i in range(len(lane1)):
                rightLane[int(math.floor(lane1[i][1] / self.slideThickness))] = lane1[i]    

        return leftLane, rightLane

    # test - thanghv
    def fillPoly(self, img):
        mask = np.zeros_like(img)
        _,cnts,__ = cv2.findContours(img, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cntSize = len(cnts)
        # for i in range(0, cntSize):
        return mask

