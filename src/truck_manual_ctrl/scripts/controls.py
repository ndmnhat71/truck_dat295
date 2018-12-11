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
#string-representations of controllers
PS3 = "ps3"
XBOX = "xbox"

DEFAULT_GAMEPAD = PS3

#buttons
LEFT_JOY_X = "left_joy_x"
A_BUTTON = "a_button"
B_BUTTON = "b_button"
X_BUTTON = "x_button"
Y_BUTTON = "y_button"
LEFT_BUMPER = "left_bumper"
LEFT_TRIGGER = "left_trigger"
RIGHT_TRIGGER = "right_trigger"
SELECT_BUTTON = "select_button"
START_BUTTON = "start_button"
DPAD_UP = "dpad_up"

#button types
AXES = 'a'
BUTTONS = 'b'

#map button to joy message indexes
GAMEPAD_MAP = {
    LEFT_JOY_X : [(AXES, 0), (AXES, 0)],
    A_BUTTON : [(BUTTONS, 14), (BUTTONS, 0)],   
    B_BUTTON : [(BUTTONS, 13), (BUTTONS, 1)], 
    X_BUTTON : [(BUTTONS, 15), (BUTTONS, 2)],
    Y_BUTTON : [(BUTTONS, 12), (BUTTONS, 3)], 
    LEFT_BUMPER : [(BUTTONS, 10), (BUTTONS, 4)],
    LEFT_TRIGGER : [(AXES, 12), (AXES, 2)],
    RIGHT_TRIGGER : [(AXES, 13), (AXES, 5)], 
    SELECT_BUTTON : [(BUTTONS, 0), (BUTTONS, 6)], 
    START_BUTTON : [(BUTTONS, 3), (BUTTONS, 7)],
    DPAD_UP: [(BUTTONS, 4), (AXES, 7)]
}

#indexes for gamepad map
gamepads = {
    PS3: 0,
    XBOX: 1
}

# driving commands
STEER = "steer"
DYNAMIC_SPEED = "dynamic"
DEAD_MANS_SWITCH = "dmg"
TOGGLE_AUTOMATIC = "auto"
TOGGLE_REVERSE = "rev"
FULL_SPEED_FORWARD = "fullAcc"
FAST_SPEED_FORWARD = "fastAcc"
SLOW_SPEED_FORWARD = "slowAcc"
FULL_SPEED_BACKWARD = "fullRev"
SLOW_SPEED_BACKWARD = "slowrev"
JOURNEY_START = "journey_start"

# control scheme
CONTROLS_MAP = {
    STEER: LEFT_JOY_X,
    DYNAMIC_SPEED: RIGHT_TRIGGER,
    DEAD_MANS_SWITCH: LEFT_TRIGGER,
    TOGGLE_AUTOMATIC: START_BUTTON,
    TOGGLE_REVERSE: SELECT_BUTTON,
    FULL_SPEED_FORWARD: A_BUTTON,
    FAST_SPEED_FORWARD: Y_BUTTON,
    SLOW_SPEED_FORWARD: X_BUTTON,
    FULL_SPEED_BACKWARD: LEFT_BUMPER,
    SLOW_SPEED_BACKWARD: B_BUTTON,
    JOURNEY_START: DPAD_UP
}

class GamepadMapFormatError(Exception):
     def __str__(self):
         return "The GAMEPAD_MAP is poorly formatted"


# returns dict with key = button, value = input
def getButtons(data, controller):
    i = gamepads[controller]
    d = {}
    for key, value in GAMEPAD_MAP.iteritems():
        (t, b) = value[i]
        if t == AXES:
            d[key] = data.axes[b]
        elif t == BUTTONS:
            d[key] = data.buttons[b]
        else:
            raise GamepadMapFormatError()
    if controller == XBOX:
        pass
        #d[LEFT_JOY_X] = d[LEFT_JOY_X] * -1
    return d
