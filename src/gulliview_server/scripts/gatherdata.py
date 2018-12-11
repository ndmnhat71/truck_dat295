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
import rospy
from custom_msgs.msg import *

from sensor_msgs.msg import Joy
import time


class GatherDataPoints:
    def __init__(self):
        rospy.init_node('gather_data', anonymous=False)
        
        self.last_point = None
        self.filename = raw_input("input file name: ")
        
        rospy.Subscriber('gv_positions', GulliViewPositions, self.gvPositionsHandler)
        rospy.Subscriber('joy', Joy, self.joyHandler)
        
        self.prev_x = 0
    
    
    def writeToFile(self):
        if self.last_point == None:
            print "last_point is None"
        else:
            print "appending: " + str(self.last_point)
            with open(self.filename, 'a') as f:
                f.write(str(self.last_point[0]) + " " + str(self.last_point[1]) + '\n')
    
    def joyHandler(self, data):
        if data.buttons[14] == 1.0 and self.prev_x == 0.0:
            self.writeToFile()
            
        self.prev_x = data.buttons[14]
            
    
    def gvPositionsHandler(self, data):
        p1 = (data.p1.x, data.p1.y)
        p2 = (data.p2.x, data.p2.y)
        tagid1 = data.tagid1
        tagid2 = data.tagid2
        cameraid = data.cameraid
        
        print cameraid
        
        self.last_point = p1
        
        


    def spin(self):
        
        while True:
            raw_input("press enter to get data point")
            
            if rospy.is_shutdown():
                break
            self.writeToFile()
        



if __name__ == '__main__':
    g = GatherDataPoints()
    g.spin()
