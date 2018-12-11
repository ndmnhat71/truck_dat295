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
import numpy as np
import cv2
import matplotlib.pyplot as plt
from os.path import dirname, abspath
from math import ceil


IMG_PATH = '/map.png'
IMG_PATH_CENTERLINE = '/map_centerline.png'
SCALE = 10  # Map img is in scale 1:10


# For representing an obstacle on the track
#
# All measurements (including coordinates) are in cm
class Obstacle:

    def __init__(self, x, y, width, height, padding):
        self.x = x  # Lower left corner
        self.y = y  # Lower left corner
        self.width = width
        self.height = height
        self.padding = padding

        self.matrix_backup = []
        self.active = False


# All measurements (including coordinates) are in cm
OBSTACLES = [
        Obstacle(110, 143, 30, 60, 1),
        Obstacle(220, 150, 30, 20, 1),
        Obstacle(147, 446, 50, 100, 1),
        Obstacle(288, 844, 30, 35, 1),
        Obstacle(200, 634, 30, 25, 1),
        Obstacle(60, 800, 20, 30, 1)
    ]


class Map:

    def __init__(self, centerline=False):
        
        if centerline:
            path = IMG_PATH_CENTERLINE
        else:
            path = IMG_PATH
            
        self.matrix = readImgToMatrix(path)
        self.scale = SCALE
        self.obstacles = OBSTACLES


    def getMapAndScale(self):
        return (self.matrix, self.scale)


    # Takes an Index value
    # Adds the corresponding Obstacle to the matrix of this Map
    #
    # If an Obstacle was added:
    #     Returns True
    # If given index is out of bounds, or the corresponding Obstacle is already activated:
    #     Returns False
    def addObstacle(self, index):

        # Checking the validity of the given index
        try:
            obstacle = self.obstacles[index]
        except IndexError:
            return False

        # If given obstacle is already active, there is no need to add it again
        if obstacle.active:
            return False

        height = int(ceil(obstacle.height))
        width = int(ceil(obstacle.width))
        padding = int(ceil(obstacle.padding))
        (x, y) = (int(obstacle.x), int(obstacle.y))

        # Going through all rows in the matrix
        for i in range(height):
            row = []

            # Going through all elements on each row
            for j in range(width):

                # For storing the old matrix element values
                row.insert(j, self.matrix[y-i][x+j])

                # If there is padding, adding a grey frame with padding width above and below the obstacle
                if i+1 <= padding or padding > (height-1 - i):
                    self.matrix[y-i][x+j] = 2
                # If there is padding, adding a grey frame with padding width to the left and right of the obstacle
                elif j+1 <= padding or padding > (width-1 - j):
                    self.matrix[y-i][x+j] = 2
                # Adding non-padding area
                else:
                    self.matrix[y-i][x+j] = 0

            # Storing the old matrix element values in 'matrix_backup' of given obstacle
            obstacle.matrix_backup.insert(i, row)
            
        obstacle.active = True
        return True


    # Takes an Index value
    # Removes the corresponding Obstacle from the matrix of this Map
    #
    # If an Obstacle was removed:
    #     Returns True
    # If given index is out of bounds, or the corresponding Obstacle is already deactivated:
    #     Returns False
    def removeObstacle(self, index):
        try:
            obstacle = self.obstacles[index]
        except IndexError:
            return False

        # If given obstacle is Not active, there is no need to reset the matrix
        if not obstacle.active:
            return False

        height = int(ceil(obstacle.height))
        width = int(ceil(obstacle.width))
        padding = int(ceil(obstacle.padding))
        (x, y) = (int(obstacle.x), int(obstacle.y))


        # Using 'matrix_backup' of given obstacle to remove the obstacle
        # and reset the affected section of the matrix:

        # Going through all rows in the matrix
        for i in range(height):

            # Going through all elements on each row
            for j in range(width):
                self.matrix[y-i][x+j] = obstacle.matrix_backup[i][j]

        # Clearing 'backup' of given obstacle
        obstacle.matrix_backup = []
        obstacle.active = False
        return True


    # Takes (x, y)-coordinates for an element
    #
    # Coordinates are assumed to be in cm
    #
    # If given coordinates are valid for the given matrix:
    #     Returns the value of element at given index,
    #     with respect to the scale of the matrix
    # If given coordinates are out of bounds:
    #     Returns None 
    def getValue(self, x, y):
        (ix, iy) = (int(x), int(y))
        try:
            return self.matrix[iy][ix]
        except IndexError:
            return None


    # Takes (x, y)-coordinates
    #
    # Coordinates are assumed to be in cm
    #
    # If there is an Obstacle at given position:
    #     Returns the index of that obstacle
    # Otherwise:
    #     Returns None
    def getObstacle(self, x, y):
        (ix, iy) = (int(x), int(y))

        for index, o in enumerate(self.obstacles):
            if ix in range(o.x, o.x + o.width) and iy in range(o.y - (o.height-1), o.y+1):
                # Obstacle found
                return index

        # No obstacle found
        return None


# Takes a path (relative to current directory) to a an image file containing a Map representation
# (path='/map.png' for file 'map.png, located in current directory)
# The file should be in '.png'-format
#
# Returns an array of rows, where each row is an array of elements
def readImgToMatrix(path):
    dirpath = dirname(abspath(__file__))
    matrix = np.asarray(cv2.imread(dirpath + path, 0), dtype=np.uint8).tolist()

    # Going through all elements, adjusting their value to match [0, 1, 2]
    # Where:
    #     0 = Black
    #     1 = White
    #     2 = Grey
    for row in range(len(matrix)):
        for elem in range(len(matrix[row])):
            # Not black
            if matrix[row][elem] != 0:
                # White
                if matrix[row][elem] == 255:
                    matrix[row][elem] = 1
                # Grey
                else:
                    matrix[row][elem] = 2

    return matrix
