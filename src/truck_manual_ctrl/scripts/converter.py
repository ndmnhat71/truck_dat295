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
from controls import *

"""
Steer rate is how fast the wheels turn
Acc rate is how fast the velocity changes when accelerating or reversing (is always positive)
Slow down rate is how fast the velocity changes when neither accelerating or reversing (is always positive)

Each rate has a constant part and a variable part.
The variable part is increased when the current speed/angle is far from the target speed/angle

The variable part varies from 0 to the constants below.
The rates below are in degrees/s or m/s^2

E.g., When the current angle is all the way to the left, and you start to steer right,
the total steer rate will be (STEER_RATE_CONSTANT + STEER_RATE_VARIABLE)

When the current angle is all the way to the right and you steer right,
the total steer rate will only be STEER_RATE_CONSTANT

When in between the steer rate will be approx. (STEER_RATE_CONSTANT + STEER_RATE_VARIABLE/2)

The same logic applies to speed

---

The variable rate is a linear equation y = kx + m, different for each rate

Example: acceleration forward rate

When at MAX_SPEED, the variable rate is 0
When ar MIN_SPEED, the variable rate is ACC_RATE_VARIABLE

we have: 0 = k*MAX_SPEED + m
         ACC_RATE_VARIABLE = k*MIN_SPEED + m

=> solve and get equation


----

In all cases a target speed/angle is calculated, and a rate.
For the new angle/speed we will try to move toward the target, but we are restricted by the rate.
This creates more smooth steering/accelerating

"""

STEER_RATE_CONSTANT = 17 #degrees per second
STEER_RATE_VARIABLE = 35#40 #in addition to constant rate. this is max rate

ACC_RATE_CONSTANT = 1.5 #m/s^2 
ACC_RATE_VARIABLE = 1.5 #in addition to constant rate. this is max rate

SLOW_DOWN_RATE_CONSTANT = 0.5 #m/s^2
SLOW_DOWN_RATE_VARIABLE = 0.5 #in addition to constant rate. this is max rate. for driving forward, backward is using forwards rate

class Converter:
    def __init__(self, joy_rate, min_angle, max_angle, min_speed, max_speed):
        
        self.prev_journey_start = False
        
        self.reverse_mode = False
        self.prev_rev_toggle_but = 0
        self.auto_mode = False
        self.prev_auto_toggle_but = 0

        self.current_steering_angle = 0 
        self.current_speed = 0 


        self.steer_k = (STEER_RATE_VARIABLE / float(joy_rate)) / (max_angle - min_angle)
        self.steer_m_r = -min_angle * self.steer_k
        self.steer_m_l = max_angle * self.steer_k
        self.steer_c = STEER_RATE_CONSTANT / float(joy_rate)

        self.acc_k = (ACC_RATE_CONSTANT / float(joy_rate)) / (max_speed - min_speed)
        self.acc_m_b = -min_speed * self.acc_k
        self.acc_m_f = max_speed * self.acc_k
        self.acc_c = ACC_RATE_CONSTANT / float(joy_rate) 

        self.slow_k = (SLOW_DOWN_RATE_VARIABLE / float(joy_rate)) / max_speed
        self.slow_c = SLOW_DOWN_RATE_CONSTANT / float(joy_rate)

        self.MIN_ANGLE = min_angle
        self.MAX_ANGLE = max_angle
        self.MIN_SPEED = min_speed
        self.MAX_SPEED = max_speed


    def getDriveCommands(self, buttons):

        #set correct mode
        self.handleReverseMode(buttons[CONTROLS_MAP[TOGGLE_REVERSE]])
        self.handleAutoMode(buttons[CONTROLS_MAP[TOGGLE_AUTOMATIC]])

        deadMansSwitch = self.hasDeadMansSwitch(buttons[CONTROLS_MAP[DEAD_MANS_SWITCH]])
        
        journey_start = self.hasJourneyStart(buttons[CONTROLS_MAP[JOURNEY_START]])
        
        
        if journey_start and (not self.prev_journey_start):
            js_ret = True
        else:
            js_ret = False
        self.prev_journey_start = journey_start
        
        
        
        #only calculate speed/angle if needed
        if not self.auto_mode and deadMansSwitch:
            newangle = self.getNewAngle(buttons[CONTROLS_MAP[STEER]])
            newspeed = self.getNewSpeed(buttons)
        else:
            newangle = 0
            newspeed = 0

        self.current_speed = newspeed
        self.current_steering_angle = newangle


        return {'angle': newangle, 
                'speed': newspeed, 
                'dms': deadMansSwitch, 
                'auto_mode': self.auto_mode, 
                'journey_start': js_ret}


    #handles flipping between reverse mode and forward mode
    def handleReverseMode(self, toggle_rev_but):
        if toggle_rev_but == 1:
            if self.prev_rev_toggle_but == 0:
                self.reverse_mode = not self.reverse_mode
                
        self.prev_rev_toggle_but = toggle_rev_but

    # handles flipping between automatic mode or manual mode
    def handleAutoMode(self, toggle_auto_but):
        if toggle_auto_but == 1:
            if self.prev_auto_toggle_but == 0:
                self.auto_mode = not self.auto_mode

        self.prev_auto_toggle_but = toggle_auto_but

    #handles the dead mans switch
    def hasDeadMansSwitch(self, dms_but):
        return dms_but < 0

    def hasJourneyStart(self, js_but):
        return js_but == 1

    def getNewAngle(self, left_joy):
        targetangle = self.getTargetAngle(left_joy)

        if targetangle > self.current_steering_angle:
            #moving left
            rate = self.leftTurnRate(self.current_steering_angle)
            return min(targetangle, self.current_steering_angle + rate)
        
        else:
            #moving right
            rate = self.rightTurnRate(self.current_steering_angle)
            return max(targetangle, self.current_steering_angle - rate)

    def getTargetAngle(self, left_joy):
            if left_joy >= 0:
                return left_joy * self.MAX_ANGLE
            else:
                return -left_joy * self.MIN_ANGLE

    def leftTurnRate(self, cur_angle):
        return self.steer_c - self.steer_k * cur_angle + self.steer_m_l

    def rightTurnRate(self, cur_angle):
        return self.steer_c + self.steer_k * cur_angle + self.steer_m_r


    def getNewSpeed(self, buttons):
        
        targetSpeed = self.getTargetSpeed(buttons)

        if targetSpeed == 0:
            rate = self.getSlowDownRate(self.current_speed)
	    print rate, self.current_speed
            if self.current_speed >= 0:
                print 1
		newspeed = max(0,self.current_speed - rate)
            else:
		print 2
                newspeed = min(0,self.current_speed + rate)
        else:
            if targetSpeed < self.current_speed:
                rate = self.getDeAccRate(self.current_speed)
                newspeed = max(targetSpeed, self.current_speed - rate)
            else:
                rate = self.getAccRate(self.current_speed)
                newspeed = min(targetSpeed, self.current_speed + rate)

        return newspeed
        
    def getTargetSpeed(self, buttons):

        ds = buttons[CONTROLS_MAP[DYNAMIC_SPEED]] #typically the right trigger
        if ds != 1:
            if not self.reverse_mode: #forward
                return ((2 - (ds + 1)) / 2.0) * self.MAX_SPEED
                
            else: #reverse
                return ((2 - (ds + 1)) / 2.0) * self.MIN_SPEED
            
        elif buttons[CONTROLS_MAP[FULL_SPEED_FORWARD]] == 1:
            return self.MAX_SPEED

        elif buttons[CONTROLS_MAP[SLOW_SPEED_BACKWARD]] == 1:
            return self.MIN_SPEED * 1.0/2

        elif buttons[CONTROLS_MAP[SLOW_SPEED_FORWARD]] == 1:
            return self.MAX_SPEED * 1.0/3

        elif buttons[CONTROLS_MAP[FAST_SPEED_FORWARD]] == 1:
            return self.MAX_SPEED * 2.0/3

        elif buttons[CONTROLS_MAP[FULL_SPEED_BACKWARD]] == 1:
            return self.MIN_SPEED
        else:
            return 0 #no button pressed
        

    def getDeAccRate(self, cur_speed):
        return self.acc_c + self.acc_k * cur_speed + self.acc_m_b

    def getAccRate(self, cur_speed):
        return self.acc_c - self.acc_k * cur_speed + self.acc_m_f

    def getSlowDownRate(self, cur_speed):
        return self.slow_c + self.slow_k * abs(cur_speed)
