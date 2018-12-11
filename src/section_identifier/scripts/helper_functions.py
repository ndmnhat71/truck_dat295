from __future__ import division
import math


def getAngleBetweenPoints(p1, p2):
    if p2.x - p1.x != 0:
        return math.atan2(p2.y - p1.y, p2.x - p1.x)
    else:
        return 1337
        # Driving along the Y-axis gives diff_X = p2.x - p1.x = 0 which is undefined for tangent-function
        # (division by 0).
        # 1337 - value is used in getActionFromRadians() to identify forward-driving
        # when driving from left -> right || right -> left


def getActionFromRadians(beta, initial_dir):
    if initial_dir == 'left':

        if -math.pi / 2 - 0.08 < beta <= -(math.pi / 3):
            return 'forward'
        elif -(math.pi / 3) < beta <= (math.pi / 3):
            return 'turn_right'
        else:
            return 'turn_left'

    if initial_dir == 'right':
        if 0 < beta <= math.pi / 3:
            return 'turn_left'
        elif -2 * math.pi / 3 < beta <= -math.pi or 2 * math.pi / 3 < beta <= math.pi:
            return 'turn_right'
        else:
            return 'forward'

    if initial_dir == 'up':
        if math.pi / 8 < beta <= (math.pi / 2):
            return 'turn_right'
        else:
            return 'turn_left'

    if initial_dir == 'down':
        if -math.pi < beta <= -(math.pi / 2):
            return 'turn_right'
        else:
            return 'turn_left'


def getDirection(p1, p2):
    dx = p2.x - p1.x
    dy = p2.y - p1.y

    beta = math.atan2(dy, dx)

    print "Direction Angle: " + str(beta)

    if -2 * math.pi / 3 < beta <= - math.pi / 6:
        return 'left'
    elif math.pi / 6 < beta <= 2 * math.pi / 3:
        return 'right'
    elif -math.pi / 6 < beta <= math.pi / 6:
        return 'up'
    else:
        return 'down'
