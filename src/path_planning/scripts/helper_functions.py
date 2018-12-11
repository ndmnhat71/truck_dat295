from Point import *
from math import *

HEADER_FRONTAXIS_TO_JOINT = 22.0
HEADER_BACKAXIS_TO_JOINT = 5.0
HEADER_LENGTH = 27.0
TRAILER_LENGTH = 44.5+10.25/2
HL_FRONT = 9.5
TL_BACK = 10.25/2 + 8.5

HEADER_WIDTH = 18;
TRAILER_WIDTH = 18

MAX_LEFT_ANGLE = -19
MAX_RIGHT_ANGLE = 16

LANE_WIDTH = 19
OUTSIDE_TURN_ERROR = 9
OTHERLANE_WEIGHT = 10
PADDING_WEIGHT = 20
DT = 25


def calculate_steering(steering_min, steering_max, dd, iters, target_error, pos, theta1, theta2, ec):
    #Calculates a point within 1 unit of the optimal path, return the closest possibility if we cant find the optimal path
    steering_new = (steering_min + steering_max)/2
    (new_point, t1, t2) = calculateNextState(theta1, theta2, pos, dd, steering_new)
    error = ec.calculateError(new_point)
    if abs(error-target_error)<0.1 or iters==0:
        return (new_point, t1, t2)
    elif error<target_error:
        #search right
        return calculate_steering(steering_new, steering_max, dd, iters-1, target_error, pos, theta1, theta2, ec)
    else:
        #search left
        return calculate_steering(steering_min, steering_new, dd, iters-1, target_error, pos, theta1, theta2, ec)


def calculateNextState(theta1, theta2, pos, dd, steering_angle_rad):

    dt1 = (dd * tan(steering_angle_rad)) / HEADER_LENGTH
    next_theta1 = theta1 + dt1

    r = HEADER_BACKAXIS_TO_JOINT
    x = sqrt(dd*dd + (r*dt1)**2)

    t1_avg = (theta1 + next_theta1)/2

    next_theta2 = theta2 + (x * sin((atan2(r*dt1, dd) + theta1 - theta2))) / TRAILER_LENGTH



    dx = pos.x - HEADER_LENGTH * cos(theta1)
    dy = pos.y - HEADER_LENGTH * sin(theta1)

    next_x = dx + dd * cos(t1_avg) + HEADER_LENGTH * cos(next_theta1)
    next_y = dy + dd * sin(t1_avg) + HEADER_LENGTH * sin(next_theta1)


    return (Point(next_x,next_y), next_theta1, next_theta2)

def rounding(x, y, th1, th2, modPoint, modTheta):

    m_x = x % modPoint
    if m_x >= modPoint/2:   #round up
        x = x-m_x + modPoint
    else:                   #round down
        x = x - m_x

    m_y = y % modPoint
    if m_y >= modPoint/2:   #round up
        y = y-m_y + modPoint
    else:                   #round down
        y = y - m_y

    th1 = round(th1, 1)
    m_t1 = round(th1 % modTheta, 1)
    if m_t1 >= modTheta/2:   #round up
        th1 = th1-m_t1 + modTheta
    else:                   #round down
        th1 = th1 - m_t1

    th2 = round(th2, 1)
    m_t2 = round(th2 % modTheta, 1)
    if m_t2 >= modTheta/2:   #round up
        th2 = th2-m_t2 + modTheta
    else:                   #round down
        th2 = th2 - m_t2

    return ((x,y),th1,th2)
    
    

def getPointsInBetween(p1, p2, n):
    p1x, p1y = p1
    p2x, p2y = p2

    dx = p2x - p1x
    dy = p2y - p1y

    stepX = dx/float(n-1)
    stepY = dy/float(n-1)

    points = []
    for i in range(0, n):
        x = int(round(p1x + i * stepX))
        y = int(round(p1y + i * stepY))
        points.append((x, y))

    return points

#Exakt some method is in auto_truck, should be placed in a central package or something
def hasPassedLine(p, (l1, l2)):

    if l1.x - l2.x !=0 and l1.y - l2.y !=0:
        slope = float(l1.y - l2.y) / float(l1.x - l2.x)
        prependularSlope = (-1)/slope
        prependularM = l2.y - l2.x*prependularSlope

        if l1.y < l2.y:
            #up
            return (p.x*prependularSlope + prependularM - p.y) < 0
        else:
            #down
            return (p.x*prependularSlope + prependularM - p.y) > 0

    elif l1.x - l2.x:
        #straight in x direction
        if l1.x < l2.x:
            #right
            return p.x > l2.x
        else:
            #left
            return p.x < l2.x

    else:
        #straight in y direction
        if l1.y < l2.y:
            #up
            return p.y > l2.y
        else:
            #down
            return p.y < l2.y


def getClosestIndex(path, (x,y)):
    if path == None:
        return 0
    i = 0
    minl = 9999
    minindex = 0
    for rx,ry in path:
        dx = rx - x
        dy = ry - y
        l = sqrt(dx**2 + dy**2)
        if l < minl:
            minl = l
            minindex = i
        i += 1
    return minindex
    
def getDistance((x1,y1), (x2, y2)):
    return sqrt((x2 - x1)**2 + (y2 - y1)**2)

def traversePath(p, path):
    path = list(path)


    if len(path) < 2:
        return path

    l1 = path.pop(0)
    l2 = path.pop(0)


    while hasPassedLine(Point(*p), (Point(l1.x, l1.y), Point(l2.x, l2.y))):


        if len(path) == 0:
            return [l2]


        l1 = l2
        l2 = path.pop(0)

    return [l1, l2] + path


