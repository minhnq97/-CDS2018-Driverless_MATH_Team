#!/usr/bin/env python
import cv2
import numpy as np
import detectshadow

class Lane:
    minThreshold = np.array([0, 0, 180])
    maxThreshold = np.array([179, 30, 255])
    slideThickness = 10
    VERTICAL = 0
    HORIZONTAL = 1
    def __init__(self):
        self.a = 1

    def imgProcess(self, src):
        # shadow = detectshadow.Shadow()
        # src = shadow.shadowRemoving(src)
        img = self.preProcess(src)
        binary = self.toBinaryImg(img)
        # greyImg = self.toGrayImg(img)
        # smoothImg = self.smoothImg(greyImg)
        # edgeDetec = self.cannyEdgeDetection(smoothImg)
        # edgeDetec = cv2.dilate(edgeDetec, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)))
        # imgfill = self.fillPoly(smoothImg)
        cv2.imshow('img', binary)
        dst = cv2.dilate(binary, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)))
        bird = self.birdViewTransform(dst)
        cv2.imshow('test', bird)
        layers = self.splitLayer(bird, self.VERTICAL)
        points = self.centerRoadSide(layers)

        for i in range(len(points)):
            for j in range(len(points[i])):
                cv2.circle(bird, points[i][j],2,(255,255,255),thickness=1, lineType=8)
        cv2.imshow('point',bird)

        #show
        # cv2.imshow('test binaryImg', greyImg)
        # cv2.imshow('test smoothImg', smoothImg)
        # cv2.imshow('test edge', edgeDetec)

    def preProcess(self, img):
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        imgThresholded = cv2.inRange(imgHSV, self.minThreshold, self.maxThreshold)
        imgHSV = cv2.bitwise_and(img, img, mask = imgThresholded )
        return imgHSV

    # Convert IMG to binary
    def toBinaryImg(self, img):
        greyImg = self.toGrayImg(img)
        thresh, res = cv2.threshold(greyImg, 160, 255, cv2.THRESH_BINARY)
        return res

    # Convert IMG to gray
    def toGrayImg(self, img):
        greyImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return greyImg

    def birdViewTransform(self, image):
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
        return warped

    # Smooth IMG    
    def smoothImg(self, img):
        # res = cv2.blur(img, (2, 2),0)
        kernel_size = 5
        res = cv2.GaussianBlur(img,(kernel_size, kernel_size),0)
        return res

    def cannyEdgeDetection(self, img):
        low_threshold = 50
        high_threshold = 50
        edges = cv2.Canny(img, low_threshold, high_threshold)
        return edges

    def splitLayer(self, img, direction):
        height, width = img.shape
        res = []
        if(direction == self.VERTICAL):
            for i in range(0,height-self.slideThickness,self.slideThickness):
                layer = img[i:i+self.slideThickness-1, :]
                res.append(layer)
        else:
            for i in range(0,width-self.slideThickness,self.slideThickness):
                layer = img[height-1, i:width+self.slideThickness-1]
                res.append(layer)
        return res
    
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
                    print (cX,cY)
                    tmp.append((cX,cY))
            res.append(tmp)
        return res

    def fillPoly(self, img):
        mask = np.zeros_like(img)
        _,cnts,__ = cv2.findContours(img, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cntSize = len(cnts)
        # for i in range(0, cntSize):
        return mask
