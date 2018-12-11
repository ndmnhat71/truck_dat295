#!/usr/bin/env python
import rospy
import rospkg
import os
import subprocess
import psutil
from subprocess import Popen
from std_msgs.msg import *
from custom_msgs.msg import *


class SectionLock:
    clients = {}

    def __init__(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        rospy.init_node('section_lock', anonymous=True)

        self.pub = rospy.Publisher('section_lock', String, queue_size=0)

        rospy.Subscriber("section_identifier", String, self.callback)

        self.handled_crossing = False
        self.old_data = None
        self.proc = None

    def callback(self, data):
        # If you try to acquire a diffent lock than the one you are holding you will release the old
        # lock

        if (self.proc is not None) and (data.data != self.old_data):
            # Tell the java zookeeper tool to release the lock
            try:
                # The process may not be alive so we add this line in a try-catch
                # to avoid exceptions. Not necessary. Just to avoid red prints in command line
                self.proc.communicate(input='\n')
            except (ValueError, OSError):
                pass
            finally:
                # Tell section_identifier that you left the section
                self.pub.publish("release")
                self.handled_crossing = False

        if not self.handled_crossing and (data.data != self.old_data):
            self.handled_crossing = True
            self.old_data = data.data
            print data

            rospack = rospkg.RosPack()
            lock_jar_path = rospack.get_path('section_lock') + '/scripts/lock_zk_node.jar'
            

            # We ask for the section lock
            env = dict(os.environ)
            env['JAVA_OPTS'] = 'foo'
            self.proc = Popen(['java', '-jar', lock_jar_path, 'localhost:2181', data.data], env=env, stdout=subprocess.PIPE, stdin=subprocess.PIPE)
            print "Asked for data: " + data.data
            # Signal the truck to stop
            self.pub.publish("stop")

            while True:
                line = self.proc.stdout.readline()

                if 'lock_accepted' in line:
                    # We've been granted the lock!
                    # Tell the truck to continue driving
                    # print "I said you can drive"
                    # self.pub.publish("continue")
                    break


if __name__ == '__main__':
    s = SectionLock()
    rospy.spin()
