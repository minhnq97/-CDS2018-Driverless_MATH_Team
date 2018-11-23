import math

import constant as const

class Car:
    leftLane = []
    rightLane = []
    def __init__(self, leftLane, rightLane):
        self.leftLane = leftLane
        self.rightLane = rightLane

    def carControl(self):
        index = len(self.leftLane) - 11
        error = 0
        while (self.leftLane[index] == None and self.rightLane[index] == None):
            index-=1
            if (index < 0):
                return
        
        if (self.leftLane[index] != None and self.rightLane[index] != None):
            error = self.getAngle(((self.leftLane[index][0] + self.rightLane[index][0]) / 2, 
                                    (self.leftLane[index][1] + self.rightLane[index][1]) / 2))
        elif (self.leftLane[index] != None):
            error = self.getAngle((self.leftLane[index][0]+20,self.leftLane[index][1]))
        else:
            error = self.getAngle((self.rightLane[index][0]-20,self.rightLane[index][1]))

        velocity = 30
        return error, velocity

    def getAngle(self, nextPoint):
        dstX = float(nextPoint[0])
        dstY = float(nextPoint[1])
        # print 'point: {},{}'.format(nextPoint[0],nextPoint[1])

        if(dstX == const.CAR_CAR_POS_X):
            return 0
        if(dstY == const.CAR_CAR_POS_Y):
            return (dstX < const.CAR_CAR_POS_X if -90 else 90)
        pi = math.acos(-1)
        dx = dstX - const.CAR_CAR_POS_X
        dy = dstY - const.CAR_CAR_POS_Y
        dy = -dy
        if(dx < 0):
            print 'error: {}; dx: {}; dy: {}'.format((math.atan(-dx/dy)*180/pi),dx,dy)
            return (-math.atan(-dx/dy)*180/pi)
        print 'error: {}; dx: {}; dy: {}'.format((math.atan(dx/dy)*180/pi),dx,dy)
        return math.atan(dx/dy)*180/pi