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
from map_func import *

import rospy
from std_msgs.msg import Int8
from os.path import dirname, abspath
from PIL import Image
import matplotlib.patches as patches
import matplotlib.text as text


PUBLISH_TOPIC = 'map_updated'
IMG_PATH = '/map.png'


# For adding matplotlib-specific plot elements to an Obstacle object
class ObstaclePlot:

    # Takes an Obstacle object
    def __init__(self, obstacle):
        self.obstacle = obstacle

        # For handling user input
        self.plot = None
        self.text = None
        
        # For plotting with matplotlib
        self.text_x = (self.obstacle.x + self.obstacle.width + 10)
        self.text_y = (self.obstacle.y - self.obstacle.height)

        self.activated_patch = patches.Rectangle(
                (self.obstacle.x, self.obstacle.y),  # Lower left corner
                self.obstacle.width,
                -self.obstacle.height,
                fc='r', ec='0.5',
                linewidth=self.obstacle.padding
            )

        self.deactivated_patch = patches.Rectangle(
                (self.obstacle.x, self.obstacle.y),  # Lower left corner
                self.obstacle.width,
                -self.obstacle.height,
                fc='b', ec='0.5',
                linewidth=self.obstacle.padding
            )


class ObstacleHandler:

    def __init__(self):
        dirpath = dirname(abspath(__file__))

        self.map_img = Image.open(dirpath + IMG_PATH)
        self.map = Map()

        self.obstacles = []
        for obstacle in self.map.obstacles:
            self.obstacles.append(ObstaclePlot(obstacle))

        # For plotting
        self.ax = None

        # Ros topic
        self.pub = rospy.Publisher(PUBLISH_TOPIC, Int8, queue_size=10)
        rospy.loginfo("Publishing on topic '%s'", PUBLISH_TOPIC)


    # Handler for 'key_press_event'
    def onKeyPress(self, event):
        if event.key.isdigit():
            if event.key == '0':
                index = 9
            else:
                index = int(event.key)-1

            # Checking if there is an obstacle in 'obstacles' that corresponds to given number,
            # only proceeding if there is
            try:
                obstacle = self.obstacles[index]
            except IndexError:
                print "There is no obstacle with identifier '%s'" % (index+1)
                return

            # If the obstacle is active: Deactivating it
            if obstacle.obstacle.active:
                # Removing obstacle from the map matrix
                self.map.removeObstacle(index)
                print "=====\nObstacle '%s' was deactivated" % (index+1)
                self.pub.publish(index)
                rospy.loginfo("Published '%s' on topic '%s'", index, PUBLISH_TOPIC)

                # Updating obstacle plot
                obstacle.plot.remove()
                obstacle.text.remove()
                obstacle.plot = self.ax.add_patch(obstacle.deactivated_patch)
                obstacle.text = self.ax.text(obstacle.text_x, obstacle.text_y, str(index+1),
                                             verticalalignment='top', color='blue', fontweight='bold')
                plt.draw()

            # If the obstacle is inactive: Activating it
            else:
                # Adding obstacle to the map matrix
                self.map.addObstacle(index)
                print "=====\nObstacle '%s' was activated" % (index+1)
                self.pub.publish(index)
                rospy.loginfo("Published '%s' on topic '%s'", index, PUBLISH_TOPIC)

                # Updating obstacle plot
                obstacle.plot.remove()
                obstacle.text.remove()
                obstacle.plot = self.ax.add_patch(obstacle.activated_patch)
                obstacle.text = self.ax.text(obstacle.text_x, obstacle.text_y, str(index+1),
                                             verticalalignment='top', color='red', fontweight='bold')
                plt.draw()


    # Lets user activate/deactivate obstacles
    # When an obstacle is activated/deactivated, the corresponding index is published on topic 'PUBLISH_TOPIC'
    def handleObstacles(self):
        xlim = self.map_img.size[0]
        ylim = self.map_img.size[1]

        fig = plt.figure()
        self.ax = plt.axes()

        # Graph settings
        plt.axis('scaled')
        plt.xlim( (0, xlim) )
        plt.ylim( (ylim, 0) )
        plt.xlabel('-axis')
        plt.ylabel('y-axis')
        plt.title('handleObstacles()')

        # Displaying map image
        img_plot = plt.imshow(self.map_img)
        # Displaying all obstacles
        for i, obstacle in enumerate(self.obstacles):
            if obstacle.obstacle.active:
                if obstacle.plot:
                    obstacle.plot.remove()
                obstacle.plot = self.ax.add_patch(obstacle.activated_patch)
                obstacle.text = self.ax.text(obstacle.text_x, obstacle.text_y, str(i+1),
                                             verticalalignment='top', color='red', fontweight='bold')
            else:
                if obstacle.plot:
                    obstacle.plot.remove()
                obstacle.plot = self.ax.add_patch(obstacle.deactivated_patch)
                obstacle.text = self.ax.text(obstacle.text_x, obstacle.text_y, str(i+1),
                                             verticalalignment='top', color='blue', fontweight='bold')

        print ("=====\nRed obstacles are activated, Blue obstacles are deactivated\n" +
               "Press the corresponding number key, to activate/deactivate an obstacle")

        fig.canvas.mpl_connect('key_press_event', self.onKeyPress)
        plt.show()


if __name__ == '__main__':
    rospy.init_node('obstacles', anonymous=True)
    handler_obj = ObstacleHandler()

    try:
        handler_obj.handleObstacles()
    except rospy.ROSInterruptException:
        pass


# rosrun truck_map obstacle_node.py
# rostopic echo map_updated
