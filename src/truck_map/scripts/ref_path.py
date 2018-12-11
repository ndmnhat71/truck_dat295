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
from shortest_path import *

import warnings
import _tkinter
from math import sin, cos, radians
import matplotlib.pyplot as plt


GRAPH_PATH = '/graph.txt'
ALT_PATHS = 50  # Maximum number of alternative paths to search for


class VehicleState:

    def __init__(self, x, y, theta1, theta2):
        self.x = x              # x, y coordinates in cm
        self.y = y
        self.theta1 = theta1    # angles in radians
        self.theta2 = theta2


class RefPath:

    def __init__(self):
        self.graph = readFileToGraph(GRAPH_PATH)
        self.path = []
        self.alt_paths = ([], 0, 0)
        self.indexes = []


    # Takes a VehicleState object, and an array of tuples of (x, y)-coordinates, assumed to be in cm
    #
    # Calculates the shortest path from vehicle position to the first coordinate point,
    # and from each coordinate point to the next
    #
    # If a path could be created:
    #     Returns a reference path in the form of an array of tuples of (x, y)-coordinates (in cm),
    #     and an array with indexes for the points on the path which coincide
    #     with the start point, and the given coordinate points
    # Otherwise:
    #     Returns [], []
    def getRefPath(self, vehicle_state, pts):

        # Finding the Node (in valid direction) which is closest to the vehicle, to use as a start point
        start_point = getClosestToVehicle(self.graph, vehicle_state)
        # Returns [], [] if the vehicle is too far away from a valid start point
        if not start_point:
            print "== ERROR: The vehicle is too far away from a valid path"
            return [], []

        self.path = []
        self.alt_paths = ([], 0, 0)
        self.indexes = []

        # Adding start point to 'pts'
        pts.insert(0, (start_point.x, start_point.y))

        # If at least one coordinate point was given
        if len(pts) > 1:
            # Adding start point to path
            self.path.append((start_point.x, start_point.y))
            self.indexes.append(0)

            # Calculating shortest path between the points
            for point in pts[1:]:
                path = shortestPath(self.graph, start_point, Point(point[0], point[1]))
                if path != None:
                    self.path += path[1:]
                    self.indexes.append(len(self.path)-1)
                    start_point = Point(self.path[-1][0], self.path[-1][1])

                # If the given coordinate points were not in range of any Nodes
                else:
                    self.path = []
                    self.indexes = []
                    print "== ERROR: Reference path out of range for %s" % str(point)
                    break

        # Printing status msg
        if self.path == []:
            print "[] returned"
        else:
            print "Path returned"
            #print "Path:", self.path

        return self.path, self.indexes


    # Takes a path (as returned by getRefPath()),
    # and indexes for the start resp. end point, for the segment which should be replaced
    #
    # If the given parameters are invalid:
    #     Returns None
    # If there is at least one alternative path between the given start and end points:
    #     Returns an array of paths, sorted in increasing order of length,
    #     with the segement between given start and end points replaced with each of the alternative paths found
    # Otherwise:
    #     Returns []
    def getAltPaths(self, path, start_index, end_index):

        # Checking validity of given indexes
        try:
            start_point = Point(*path[start_index])
            end_point = Point(*path[end_index])
        except IndexError:
            return None

        alt_paths = altPaths(self.graph, start_point, end_point, ALT_PATHS)

        if alt_paths:
            for i, alt in enumerate(alt_paths):
                alt_paths[i] = path[:start_index] + alt + path[end_index+1:]
        else:
            alt_paths = []
        
        self.path = path
        self.alt_paths = (alt_paths, start_index, end_index)
        return alt_paths


    # Takes a path (as returned by getRefPath()),
    # indexes for the start resp. end point for the segment which should be replaced,
    # and a number (>= 1) specifying which alternative path to use
    # (nth=1 for the first alternative path, nth=2 for the second, etc.)
    #
    # If the given parameters are invalid:
    #     Returns None
    # If there is a nth alternative path between the given start and end points:
    #     Returns the given path, with the segement between given start and end points replaced with an alternative path
    # Otherwise:
    #     Returns []
    def getAltPath(self, path, start_index, end_index, nth):

        # Checking validity of given nth-value
        if nth < 1:
            return None

        alt = self.alt_paths

        # If the complete set of alternative paths has already been computed for the given parameters,
        # no need to re-compute
        if path == self.path and alt[0] and start_index == alt[1] and end_index == alt[2]:
            alt_paths = alt[0]
        else:
            alt_paths = self.getAltPaths(path, start_index, end_index)

        try:
            return alt_paths[nth-1]
        except IndexError:
            return []
