#!/usr/bin/env python

from math import sqrt, atan2
import os
from collections import deque
import copy

class Point:
    def __init__(self, x , y):
        self.x = x
        self.y = y


class errorCalc:
    def __init__(self, startList):

        self.queue = deque(startList)
        self.p1= self.queue.popleft()
        self.p2= self.queue.popleft()
        self.line = (self.p1,self.p2)

    def calculateError(self, p0):
        (self.p1,self.p2) = self.line
        while self.isAboveEnd(self.p1,self.p2,p0) and len(self.queue)>0:
            tempP1=self.p2
            tempP2=self.queue.popleft()
            self.line= (tempP1,tempP2)
            (self.p1,self.p2) = self.line

        isLeft = ((self.p2.x - self.p1.x)*(p0.y - self.p1.y) - (self.p2.y - self.p1.y)*(p0.x - self.p1.x)) >0 #decides if the error is to the left of centerline or not
        value = abs((self.p2.x - self.p1.x)*(self.p1.y-p0.y) - (self.p1.x-p0.x)*(self.p2.y-self.p1.y)) / (sqrt((self.p2.x-self.p1.x)*(self.p2.x-self.p1.x) + (self.p2.y-self.p1.y)*(self.p2.y-self.p1.y)))
        if(isLeft):
            return -value
        else:
            return value

    def isAboveEnd (self,begin, end, p0):
        #checks if a point is passed the end point of a line.
        if begin.x - end.x !=0 and begin.y - end.y !=0:
            slope = float(begin.y - end.y) / float(begin.x - end.x)
            prependularSlope = (-1)/slope
            prependularM = end.y - end.x*prependularSlope
            if begin.y < end.y:
                #going up
                return (p0.x*prependularSlope + prependularM - p0.y) < 0
            else:
                #going down
                return (p0.x*prependularSlope + prependularM - p0.y) > 0
        elif begin.x - end.x:
            #going straight in x direction
            if begin.x < end.x:
                #going right
                return p0.x > end.x
            else:
                #going left
                return p0.x < end.x
        else:
            #going straight in y direction
            if begin.y < end.y:
                #going up
                return p0.y > end.y
            else:
                #going down
                return p0.y < end.y

    def getDirection(self):
        dy = self.line[1].y - self.line[0].y
        dx = self.line[1].x - self.line[0].x
        theta = atan2(dy, dx)
        return theta

    def isAtEnd(self):
        if len(self.queue)==0:
            return True
        else:
            return False

    def getMaxDistPoint(self, point):
        p1 = self.line[1]
        p0 = self.line[0]
        d1 = sqrt( (point.x - p1.x)**2 + (point.y - p1.y)**2 )
        d0 = sqrt( (point.x - p0.x)**2 + (point.y - p0.y)**2 )
        return max(d1,d0)

    def getCopy(self):
        q = copy.copy(self.queue)
        q.appendleft(self.line[1])
        q.appendleft(self.line[0])
        return errorCalc(q)

    def is_next_Left(self):
        if len(self.queue)>0:
            isLeft = ((self.p2.x - self.p1.x)*(self.queue[0].y - self.p1.y) - (self.p2.y - self.p1.y)*(self.queue[0].x - self.p1.x)) >0
        else:
            isLeft=True
        return isLeft


    def printEC(self):
        print "EC"
        print
        for p in self.queue:
            print p.x, p.y
