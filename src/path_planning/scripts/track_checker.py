import model
import numpy as np
import cv2
from Point import Point
from math import radians
from helper_functions import *

class trackChecker:

    def __init__(self, mapp):
        self.model = model.truck() #model used to calculate error
        self.map = mapp #map with allowed/not allowed areas
        

    def checkIfInTrack2(self, toPoint, th1, th2):


        #check the range of the matrix with the allowed positions, to avoid index error
        if toPoint.x <0 or toPoint.y <0 or toPoint.x >540 or toPoint.y >950:
            return False

        points = self.model.calculateCorners(toPoint, th1, th2)

        if self.map[int(toPoint.y)][int(toPoint.x)]==0:
            return False


        right_back_wheel = Point(points[5][0], points[5][1])
        left_back_wheel = Point(points[4][0], points[4][1])
        right_front_wheel = Point(points[1][0], points[1][1])
        left_front_wheel = Point(points[0][0], points[0][1])

        #check right front wheels
        if right_front_wheel.x <0 or right_front_wheel.y <0 or right_front_wheel.x >540 or right_front_wheel.y >950:
            return False
        if self.map[int(right_front_wheel.y)][int(right_front_wheel.x)] ==0:
            return False
        #check left front wheels
        if left_front_wheel.x <0 or left_front_wheel.y <0 or left_front_wheel.x >540 or left_front_wheel.y >950:
            return False
        if self.map[int(left_front_wheel.y)][int(left_front_wheel.x)] ==0:
            return False


        #check right front wheels
        if right_back_wheel.x <0 or right_back_wheel.y <0 or right_back_wheel.x >540 or right_back_wheel.y >950:
            return False
        if self.map[int(right_back_wheel.y)][int(right_back_wheel.x)] ==0:
            return False
        #check left front wheels
        if left_back_wheel.x <0 or left_back_wheel.y <0 or left_back_wheel.x >540 or left_back_wheel.y >950:
            return False
        if self.map[int(left_back_wheel.y)][int(left_back_wheel.x)] ==0:
            return False

        return True





    def checkIfInTrack(self, prevPoint, prevth1, prevth2, toPoint, th1, th2, dt, front_ec, back_ec):
        
        #used to avoid going wrong direction, optimal path should be close enugh that this restriction holds
        if front_ec.getMaxDistPoint(toPoint) > 80:
            return (False,True)

        inPadding = False

        #check the range of the matrix with the allowed positions, to avoid index error
        if toPoint.x <0 or toPoint.y <0 or toPoint.x >540 or toPoint.y >950:
            return (False, True)

        if self.map[int(toPoint.y)][int(toPoint.x)]==0:
            return (False, True)
        #if self.map[int(toPoint.y)][int(toPoint.x)]==2:
        #    inPadding = True

        points = self.model.calculateCorners(toPoint, th1, th2)



        #header
        right_back_wheel = Point(points[5][0], points[5][1])
        left_back_wheel = Point(points[4][0], points[4][1])
        right_front_wheel = Point(points[1][0], points[1][1])
        left_front_wheel = Point(points[0][0], points[0][1])

        left_front = Point(points[6][0], points[6][1])
        right_front = Point(points[7][0], points[7][1])
        #trailer
        left_back = Point(points[8][0], points[8][1])
        right_back = Point(points[9][0], points[9][1])



        prev_points = self.model.calculateCorners(prevPoint, prevth1, prevth2)
        prev_right_back_wheel = Point(prev_points[5][0], prev_points[5][1])
        prev_left_back_wheel = Point(prev_points[4][0], prev_points[4][1])
        prev_right_front_wheel = Point(prev_points[1][0], prev_points[1][1])
        prev_left_front_wheel = Point(prev_points[0][0], prev_points[0][1])

        prev_left_front = Point(prev_points[6][0], prev_points[6][1])
        prev_right_front = Point(prev_points[7][0], prev_points[7][1])
        prev_left_back = Point(prev_points[8][0], prev_points[8][1])
        prev_right_back = Point(prev_points[9][0], prev_points[9][1])

        nbr_points = dt/4
        #TODO: affects performance quite a lot with dt amount of points
        between_back_wheel_right = getPointsInBetween((right_back_wheel.x, right_back_wheel.y), (prev_right_back_wheel.x, prev_right_back_wheel.y), nbr_points)
        between_back_wheel_left = getPointsInBetween((left_back_wheel.x, left_back_wheel.y), (prev_left_back_wheel.x, prev_left_back_wheel.y), nbr_points)

        between_front_wheel_right = getPointsInBetween((right_front_wheel.x, right_front_wheel.y), (prev_right_front_wheel.x, prev_right_front_wheel.y), nbr_points)
        between_front_wheel_left = getPointsInBetween((left_front_wheel.x, left_front_wheel.y), (prev_left_front_wheel.x, prev_left_front_wheel.y), nbr_points)


        between_front_right = getPointsInBetween((right_front.x, right_front.y), (prev_right_front.x, prev_right_front.y), nbr_points)
        between_front_left = getPointsInBetween((left_front.x, left_front.y), (prev_left_front.x, prev_left_front.y), nbr_points)


        between_back_right = getPointsInBetween((right_back.x, right_back.y), (prev_right_back.x, prev_right_back.y), nbr_points)
        between_back_left = getPointsInBetween((left_back.x, left_back.y), (prev_left_back.x, prev_left_back.y), nbr_points)


        right_front_inPadding = False
        left_front_inPadding = False
        right_back_inPadding = False
        left_back_inPadding = False

        #check right back wheel
        for (x,y) in between_back_wheel_right:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x] ==0:
                return (False, True)
            if self.map[y][x] ==2:
                right_front_inPadding = True
        #Check left back wheel
        for (x,y) in between_back_wheel_left:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x]==0:
                return (False, True)
            if self.map[y][x]==2:
                left_front_inPadding = True

        #check right front wheel
        for (x,y) in between_front_wheel_right:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x] ==0:
                return (False, True)
            if self.map[y][x] ==2:
                right_front_inPadding = True

        #check left front wheel
        for (x,y) in between_front_wheel_left:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x] ==0:
                return (False, True)
            if self.map[y][x] ==2:
                left_front_inPadding = True

        #trailer back
        for (x,y) in between_back_right:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x] ==0:
                return (False, True)
            if self.map[y][x] ==2:
                right_back_inPadding = True

        #trailer back
        for (x,y) in between_back_left:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x]==0:
                return (False, True)
            if self.map[y][x]==2:
                left_back_inPadding = True

        #check right front wheel
        for (x,y) in between_front_right:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x] ==0:
                return (False, True)
            if self.map[y][x] ==2:
                inPadding = True

        #check left front wheel
        for (x,y) in between_front_left:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x] ==0:
                return (False, True)
            if self.map[y][x] ==2:
                inPadding = True

         #calculate avarege error for key wheels

        right_front_wheel_err = front_ec.calculateError(right_front) - HEADER_WIDTH/2
        left_front_wheel_err = front_ec.calculateError(left_front) + HEADER_WIDTH/2
        right_back_wheel_err = back_ec.calculateError(right_back_wheel) - TRAILER_WIDTH/2
        left_back_wheel_err = back_ec.calculateError(left_back_wheel) + TRAILER_WIDTH/2

        if abs(right_front_wheel_err) > LANE_WIDTH/2:
            right_front_wheel_err = right_front_wheel_err * OTHERLANE_WEIGHT
        if right_front_inPadding:

            right_front_wheel_err = (abs(right_front_wheel_err)+20) * PADDING_WEIGHT
        if abs(left_front_wheel_err) > LANE_WIDTH/2:
            left_front_wheel_err = left_front_wheel_err * OTHERLANE_WEIGHT
        if left_front_inPadding:
            left_front_wheel_err = (abs(left_front_wheel_err)+20) * PADDING_WEIGHT
        if abs(right_back_wheel_err) > LANE_WIDTH/2:
            right_back_wheel_err = right_back_wheel_err * OTHERLANE_WEIGHT
        if right_back_inPadding:
            right_back_wheel_err = (abs(right_back_wheel_err)+20) * PADDING_WEIGHT
        if abs(left_back_wheel_err) > LANE_WIDTH/2:
            left_back_wheel_err = left_back_wheel_err * OTHERLANE_WEIGHT
        if left_back_inPadding:
            left_back_wheel_err = (abs(left_back_wheel_err)+20) * PADDING_WEIGHT

#        inn = False

#        if abs(right_front_wheel_err) > LANE_WIDTH/2:
#            inn = True
#        if abs(left_front_wheel_err) > LANE_WIDTH/2:
#            inn = True
#        if abs(right_back_wheel_err) > LANE_WIDTH/2:
#            inn = True
#        if abs(left_back_wheel_err) > LANE_WIDTH/2:
#            inn = True

        totError = abs(right_front_wheel_err) + abs(left_front_wheel_err) + abs(right_back_wheel_err) + abs(left_back_wheel_err)
#        if inn:
#            totError = totError * OTHERLANE_WEIGHT

        return (True, totError)

    

    def setMap(self, mapp):
        self.map = mapp
