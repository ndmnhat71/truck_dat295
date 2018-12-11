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
from ackermann_msgs.msg import AckermannDrive
#from truck_hw_api import interpolate
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from math import *
from converter import *
from controls import *

# Subscribes to joy messages and publish appropriate steering and speed commands,
# Based on a control scheme. Also handles safety buttons and a switch for automatic driving
class GamepadNode:
    def __init__(self):
        self.no_dms_count = 0
        
        print "sleeping for 1 sec"
        rospy.sleep(1)
        
        min_angle = rospy.get_param('min_angle', -21)
        max_angle = rospy.get_param('max_angle', 16)
        min_speed = rospy.get_param('min_speed', -1)
        max_speed = rospy.get_param('max_speed', 1.4)
        gamepad_rate = rospy.get_param('gamepad/rate', 50)

        self.converter = Converter(gamepad_rate, min_angle, max_angle, min_speed, max_speed)

        self.gamepad = rospy.get_param('gamepad/type', DEFAULT_GAMEPAD).lower()

        if not self.gamepad in gamepads.keys():
            self.gamepad = DEFAULT_GAMEPAD

        self.manualDrivePublisher = rospy.Publisher('man_drive', AckermannDrive, queue_size=10)
        self.autoCtrlPublisher = rospy.Publisher('auto_ctrl', Bool, queue_size=10)
        self.dmsPublisher = rospy.Publisher('dead_mans_switch', Bool, queue_size=10)
        self.journeyStartPublisher = rospy.Publisher('start_journey', Bool, queue_size=10)

        rospy.init_node('gamepad', anonymous=False)
        rospy.Subscriber('joy', Joy, self.callback)
        rospy.loginfo("init done, subscribed to /joy and publishes to several topics")

    def callback(self,data):
        #dict with key = button, value = input
        try:
            #raises gamepad map format error
            buttons = getButtons(data, self.gamepad)

            #convert button input to driving commands, etc
            commands = self.converter.getDriveCommands(buttons)

            js_ret = commands['journey_start']
            newSpeed = commands['speed']
            newAngle = commands['angle']
            deadMansSwitch = commands['dms']
            autoCtrl = commands['auto_mode']
            
            
            if js_ret:
                js_msg = Bool()
                js_msg.data = True
                self.journeyStartPublisher.publish(js_msg)
            

            dms = Bool()
            dms.data = deadMansSwitch
            self.dmsPublisher.publish(dms)
            
            ac = Bool()
            ac.data = autoCtrl
            self.autoCtrlPublisher.publish(ac)

            #only publish if needed
            if deadMansSwitch and (not autoCtrl):
                ack = AckermannDrive()
                ack.steering_angle = newAngle
                ack.speed = newSpeed
                self.manualDrivePublisher.publish(ack)
        except GamepadMapFormatError:
            rospy.logfatal("%s, shutting down", GamepadMapFormatError.message)
            exit(0)

if __name__ == '__main__':
    j = GamepadNode()
    rospy.spin()
