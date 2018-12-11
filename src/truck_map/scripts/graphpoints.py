#!/usr/bin/env python
"""
Copyright (c) 2017, Lars Niklasson
Copyright (c) 2017, Filip Slottner Seholm
Copyright (c) 2017, Fanny Sandblom
Copyright (c) 2017, Kevin Hoogendijk
Copyright (c) 2017, Nils Andren
Copyright (c) 2017, Alicia Gil Martin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Chalmers University of Technology nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
from graph_func import *

from numpy import cos, sin, arccos, ones, vstack
from numpy.linalg import lstsq
import numpy as np
from pylab import *
from math import ceil


"""

OBS!

This file is not meant to be used regularly.

It mainly consists of measured and hardcoded Graph data, which is only meant to be used once 
- to create a Graph which is then stored in a savefile.

It can then serve as backup if the svaefile should be lost, or be used to edit the Graph.

"""

SCALE = 10

ARC_DIST = 100
LIN_DIST = 200


p1 = [Point(770, 2200), Point(770, 7970)]
p2 = [Point(1160, 7970), Point(1170, 2200)]

p3 = [Point(1500, 1100), Point(770, 2200)]      # R: 1560, center = Point(2310, 2430), Neg
p4 = [Point(1170, 2200), Point(1680, 1470)]     # R: 1140, center = Point(2280, 2440), Neg

p5 = [Point(2860, 1110), Point(1500, 1100)]     # R: 1560, center = Point(2170, 2510), Neg
p6 = [Point(1680, 1470), Point(2680, 1470)]     # R: 1130, center = Point(2180, 2480), Neg

p7 = [Point(3650, 2250), Point(2860, 1110)]     # R: 1560, center = Point(2110, 2480), Neg
p8 = [Point(2680, 1470), Point(3260, 2250)]     # R: 1130, center = Point(2160, 2470), Neg

p9 = [Point(3680, 3150), Point(3650, 2250)]
p10 = [Point(3260, 2250), Point(3280, 3150)]

p11 = [Point(3630, 7850), Point(3690, 4840)]
p12 = [Point(3300, 4850), Point(3230, 7850)]

p13 = [Point(2690, 9140), Point(3630, 7850)]    # R: 1440, center = Point(2190, 7790), Pos
p14 = [Point(3230, 7850), Point(2570, 8750)]    # R: 1090, center = Point(2150, 7750), Pos

p15 = [Point(1550, 9140), Point(2690, 9140)]    # R: 1720, center = Point(2120, 7520), Pos
p16 = [Point(2570, 8750), Point(1780, 8750)]    # R: 1190, center = Point(2170, 7630), Pos

p17 = [Point(770, 7970), Point(1550, 9140)]     # R: 1210, center = Point(1980, 8010), Pos
p18 = [Point(1780, 8750), Point(1160, 7970)]    # R: 880, center = Point(2040, 7910), Pos

p191 = [Point(2620, 3820), Point(1170, 3810)]
p192 = [Point(1170, 3810), Point(770, 3810)]
p201 = [Point(770, 4200), Point(1170, 4200)]
p202 = [Point(1170, 4200), Point(2620, 4190)]

p211 = [Point(3660, 6320), Point(3270, 6320)]
p212 = [Point(3270, 6320), Point(1160, 6300)]
p213 = [Point(1160, 6300), Point(770, 6300)]
p221 = [Point(770, 6690), Point(1160, 6690)]
p222 = [Point(1160, 6690), Point(3260, 6710)]
p223 = [Point(3260, 6710), Point(3650, 6710)]

p23 = [Point(3680, 3150), Point(3280, 3150)]    # R: 870, center = Point(3470, 4000)
p24 = [Point(3280, 3150), Point(2620, 3820)]    # R: 870, center = Point(3470, 4000)
p251 = [Point(2620, 3820), Point(2600, 4000)]    # R: 870, center = Point(3470, 4000)
p252 = [Point(2600, 4000), Point(2620, 4190)]    # R: 870, center = Point(3470, 4000)
p26 = [Point(2620, 4190), Point(3300, 4850)]    # R: 870, center = Point(3470, 4000)
p27 = [Point(3300, 4850), Point(3690, 4840)]    # R: 870, center = Point(3470, 4000)
p28 = [Point(3690, 4840), Point(4310, 4210)]    # R: 870, center = Point(3470, 4000)
p291 = [Point(4310, 4210), Point(4340, 4000)]    # R: 870, center = Point(3470, 4000)
p292 = [Point(4340, 4000), Point(4320, 3810)]    # R: 870, center = Point(3470, 4000)
p30 = [Point(4320, 3810), Point(3680, 3150)]    # R: 870, center = Point(3470, 4000)


def makeGraph():

    p1_ = makeLine(p1[0], p1[1], ceil(getLength(p1[0], p1[1])/LIN_DIST))
    p2_ = makeLine(p2[0], p2[1], ceil(getLength(p2[0], p2[1])/LIN_DIST))

    p3_ = makeArc(p3[0], p3[1], Point(2310, 2430), ceil(getLength(p3[0], p3[1])/ARC_DIST), 1560, False)
    p4_ = makeArc(p4[0], p4[1], Point(2280, 2440), ceil(getLength(p4[0], p4[1])/ARC_DIST), 1140, False)
    p5_ = makeArc(p5[0], p5[1], Point(2170, 2510), ceil(getLength(p5[0], p5[1])/ARC_DIST), 1560, False)
    p6_ = makeArc(p6[0], p6[1], Point(2180, 2480), ceil(getLength(p6[0], p6[1])/ARC_DIST), 1130, False)
    p7_ = makeArc(p7[0], p7[1], Point(2110, 2480), ceil(getLength(p7[0], p7[1])/ARC_DIST), 1560, False)
    p8_ = makeArc(p8[0], p8[1], Point(2160, 2470), ceil(getLength(p8[0], p8[1])/ARC_DIST), 1130, False)

    p9_ = makeLine(p9[0], p9[1], ceil(getLength(p9[0], p9[1])/LIN_DIST))
    p10_ = makeLine(p10[0], p10[1], ceil(getLength(p10[0], p10[1])/LIN_DIST))
    p11_ = makeLine(p11[0], p11[1], ceil(getLength(p11[0], p11[1])/LIN_DIST))
    p12_ = makeLine(p12[0], p12[1], ceil(getLength(p12[0], p12[1])/LIN_DIST))

    p13_ = makeArc(p13[0], p13[1], Point(2190, 7790), ceil(getLength(p13[0], p13[1])/ARC_DIST), 1440, True)
    p14_ = makeArc(p14[0], p14[1], Point(2150, 7750), ceil(getLength(p14[0], p14[1])/ARC_DIST), 1090, True)
    p15_ = makeArc(p15[0], p15[1], Point(2120, 7520), ceil(getLength(p15[0], p15[1])/ARC_DIST), 1720, True)
    p16_ = makeArc(p16[0], p16[1], Point(2170, 7630), ceil(getLength(p16[0], p16[1])/ARC_DIST), 1190, True)
    p17_ = makeArc(p17[0], p17[1], Point(1980, 8010), ceil(getLength(p17[0], p17[1])/ARC_DIST), 1210, True)
    p18_ = makeArc(p18[0], p18[1], Point(2040, 7910), ceil(getLength(p18[0], p18[1])/ARC_DIST), 880, True)

    p191_ = makeLine(p191[0], p191[1], ceil(getLength(p191[0], p191[1])/LIN_DIST))
    p192_ = makeLine(p192[0], p192[1], ceil(getLength(p192[0], p192[1])/LIN_DIST))
    p201_ = makeLine(p201[0], p201[1], ceil(getLength(p201[0], p201[1])/LIN_DIST))
    p202_ = makeLine(p202[0], p202[1], ceil(getLength(p202[0], p202[1])/LIN_DIST))
    p211_ = makeLine(p211[0], p211[1], ceil(getLength(p211[0], p211[1])/LIN_DIST))
    p212_ = makeLine(p212[0], p212[1], ceil(getLength(p212[0], p212[1])/LIN_DIST))
    p213_ = makeLine(p213[0], p213[1], ceil(getLength(p213[0], p213[1])/LIN_DIST))
    p221_ = makeLine(p221[0], p221[1], ceil(getLength(p221[0], p221[1])/LIN_DIST))
    p222_ = makeLine(p222[0], p222[1], ceil(getLength(p222[0], p222[1])/LIN_DIST))
    p223_ = makeLine(p223[0], p223[1], ceil(getLength(p223[0], p223[1])/LIN_DIST))

    p23_ = makeArc(p23[0], p23[1], Point(3470, 4000), ceil(getLength(p23[0], p23[1])/ARC_DIST), 870, False)
    p24_ = makeArc(p24[0], p24[1], Point(3470, 4000), ceil(getLength(p24[0], p24[1])/ARC_DIST), 870, False)
    p251_ = makeArc(p251[0], p251[1], Point(3470, 4000), ceil(getLength(p251[0], p251[1])/ARC_DIST), 870, False)
    p252_ = makeArc(p252[0], p252[1], Point(3470, 4000), ceil(getLength(p252[0], p252[1])/ARC_DIST), 870, True)
    p26_ = makeArc(p26[0], p26[1], Point(3470, 4000), ceil(getLength(p26[0], p26[1])/ARC_DIST), 870, True)
    p27_ = makeArc(p27[0], p27[1], Point(3470, 4000), ceil(getLength(p27[0], p27[1])/ARC_DIST), 870, True)
    p28_ = makeArc(p28[0], p28[1], Point(3470, 4000), ceil(getLength(p28[0], p28[1])/ARC_DIST), 870, True)
    p291_ = makeArc(p291[0], p291[1], Point(3470, 4000), ceil(getLength(p291[0], p291[1])/ARC_DIST), 870, True)
    p292_ = makeArc(p292[0], p292[1], Point(3470, 4000), ceil(getLength(p292[0], p292[1])/ARC_DIST), 870, False)
    p30_ = makeArc(p30[0], p30[1], Point(3470, 4000), ceil(getLength(p30[0], p30[1])/ARC_DIST), 870, False)


    p192_.append(Point(770, 4260))
    p201_.insert(0, Point(770, 4054))
    p202_.insert(0, Point(1166, 4260))
    p191_.append(Point(1167, 3642))

    p213_.append(Point(770, 6733)) 
    p221_.insert(0, Point(770, 6527))
    p222_.insert(0, Point(1162, 6733))
    p212_.append(Point(1163, 6115))

    p223_.append(Point(3661, 6244))
    p211_.insert(0, Point(3657, 6445))
    p212_.insert(0, Point(3267, 6250))
    p222_.append(Point(3253, 6850))


    gs = [pointsToGraphMM(p1_), pointsToGraphMM(p2_), pointsToGraphMM(p3_), pointsToGraphMM(p4_), pointsToGraphMM(p5_),
          pointsToGraphMM(p6_), pointsToGraphMM(p7_), pointsToGraphMM(p8_), pointsToGraphMM(p9_), pointsToGraphMM(p10_),
          pointsToGraphMM(p11_), pointsToGraphMM(p12_), pointsToGraphMM(p13_), pointsToGraphMM(p14_), pointsToGraphMM(p15_),
          pointsToGraphMM(p16_), pointsToGraphMM(p17_), pointsToGraphMM(p18_), pointsToGraphMM(p191_), pointsToGraphMM(p192_),
          pointsToGraphMM(p201_), pointsToGraphMM(p202_), pointsToGraphMM(p211_), pointsToGraphMM(p212_), pointsToGraphMM(p213_),
          pointsToGraphMM(p221_), pointsToGraphMM(p222_), pointsToGraphMM(p223_), pointsToGraphMM(p23_), pointsToGraphMM(p24_),
          pointsToGraphMM(p251_), pointsToGraphMM(p252_), pointsToGraphMM(p26_), pointsToGraphMM(p27_), pointsToGraphMM(p28_),
          pointsToGraphMM(p291_), pointsToGraphMM(p292_), pointsToGraphMM(p30_)]


    graph = Graph()
    for g in gs:
        graph.addGraph(g)

    return graph


# HELP FUNCTIONS ------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

def getLength(from_point, to_point):
    return sqrt((to_point.x - from_point.x)**2 + (to_point.y - from_point.y)**2)

def parametricCircle(t, center, radius, positive):
    x = center.x + radius*cos(t)
    y = center.y + radius*sin(t) if positive else center.y - radius*sin(t)
    return x, y

def invParametricCircle(x, center_x, radius):
    t = arccos((float(x)-float(center_x))/float(radius))
    return t

def makeArc(start, end, center, nbr_of_points, radius, positive):
    start_t = invParametricCircle(start.x, center.x, radius)
    end_t   = invParametricCircle(end.x, center.x, radius)
    arc_T = np.linspace(start_t, end_t, nbr_of_points)

    (xs, ys) = parametricCircle(arc_T, center, radius, positive)

    points = []
    for i in range(len(xs)):
        points.append(Point(int(xs[i]), int(ys[i])))

    points[0] = start
    points[len(points)-1] = end
    return points

def makeLine(start, end, nbr_of_points):
    horizontal = abs(start.x - end.x) > abs(start.y - end.y)

    x_coords = [start.x, end.x]
    y_coords = [start.y, end.y]

    A = vstack([x_coords, ones(len(x_coords))]).T
    m, c = lstsq(A, y_coords)[0]

    # y = mx + c
    # x = y/m - c/m

    if horizontal:
        xs = np.linspace(start.x, end.x, nbr_of_points)
        if start.y == end.y:
            ys = np.linspace(start.y, end.y, nbr_of_points)
        else:
            ys = []
            for x in xs:
                y = m*x + c
                ys.append(y)
    else:
        ys = np.linspace(start.y, end.y, nbr_of_points)
        if start.x == end.x:
            xs = np.linspace(start.x, end.x, nbr_of_points)
        else:
            xs = []
            for y in ys:
                x = y/m - c/m
                xs.append(x)

    points = []
    for i in range(len(xs)):
        points.append(Point(int(xs[i]), int(ys[i])))

    points[0] = start
    points[len(points)-1] = end
    return points
