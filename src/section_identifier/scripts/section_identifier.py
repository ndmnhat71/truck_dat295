#!/usr/bin/env python
import rospy
from helper_functions import *
from std_msgs.msg import *
from custom_msgs.msg import *


class SectionIdentifier:

    def __init__(self):
        rospy.init_node('section_identifier', anonymous=True)

        self.pub = rospy.Publisher('section_identifier', String, queue_size=0)

        rospy.Subscriber("truck_state", TruckState, self.callback)

        self.msg = None
        self.msg_old = None

    def callback(self, data):

        x = data.p.x
        y = data.p.y

        if 600 < x < 1800 and 5400 < y < 7300:
            self.msg = "Intersection_1"

        elif 600 < x < 1800 and 2800 < y < 4800:
            self.msg = "Intersection_2"

        elif 2000 < x < 3850 and 2300 < y <= 5700:
            self.msg = "Roundabout"

        elif 2600 < x < 3850 and 5700 < y < 7500:
            self.msg = "Intersection_3"
        else:
            self.msg = "road"

        if self.msg != "" and self.msg != self.msg_old:
            print self.msg
            self.pub.publish(self.msg)
            self.msg_old = self.msg


if __name__ == '__main__':
    s = SectionIdentifier()

    # spin() simply keeps python from exiting until this node is stopped
rospy.spin()
