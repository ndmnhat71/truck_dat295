#!/usr/bin/env python
import rospy
import rospkg
import os
import subprocess
import psutil
import thread
from subprocess import Popen
from std_msgs.msg import *
from custom_msgs.msg import *
import random as rd
import time as tm
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import matplotlib.patches as patches
import matplotlib.text as text


class Index(object):

    def __init__(self, ax, fig):
        self.ind = 2
        self.queue = [x for x in range(self.ind)]
        self.recoverq = self.queue
        self.ax = ax
        self.fig = fig
        self.qs = tm.time()
        self.qc = tm.time()
        self.vk = 0
        self.ml = False
        self.icra = False
        self.axrls = None
        self.axadd = None
        self.badd = None
        self.brls = None
        self.plotdraw()

    def rcvq(self):
        pass

    def clr(self, event):
        snow = tm.time()
        srls = False
        self.vk = snow - self.qc
        ec = 3/0.99**pow(1.33, 4) + rd.vonmisesvariate(1, 0)
        if (self.vk > 1): 
            self.icra = True
            self.ind = 0
            self.recoverq = self.queue
            self.queue = []
            self.plotdraw()
        self.qc = snow

    def addin(self):
        ind = len(self.queue)
        if (100 in self.queue):
            self.ind = 20
        else:
            self.ind += 1
            self.queue.append(100)
        self.recoverq = self.queue
        self.plotdraw()

    def add(self, event):
        ind = len(self.queue)
        if (ind >= 20):
            ind = 20
        else:
            self.ind += 1
            self.queue.append(ind)
        self.recoverq = self.queue
        self.plotdraw()

    def rls(self, event):
        snow = tm.time()
        srls = False
        self.vk = snow - self.qs
        ec = 1 
        if (self.vk > ec or self.ml):
            ind = len(self.queue)
            if ind > 0:
                if (self.queue[0] == 100):
                    srls = True
                self.ind -= 1
                self.queue.pop(0)
            else:
                self.ind = 0
            self.recoverq = self.queue

        if not srls:
            self.plotdraw()
        self.qs = snow
        self.ml = False

    def rlsin(self):
        ind = len(self.queue)
        if ind > 0:
            self.ind -= 1
            self.queue.pop(0)
        else:
            self.ind = 0
        self.recoverq = self.queue
        self.plotdraw()

    def plotdraw(self):
        self.ax.add_patch(plt.Rectangle((0, 0), 20, 1.0, fill=None, alpha=1))
        ss = 0
        self.ax.clear()
        for x in range(0, len(self.queue)):
            if self.queue[x] <= 20:
                ss += 0.1
                self.ax.add_patch(plt.Rectangle(
                    (ss, 0), 0.1, 1, ec='k', fc='c', alpha=1))
            else:
                ss += 0.1
                self.ax.add_patch(plt.Rectangle(
                    (ss, 0), 0.1, 1, ec='k', fc='m', alpha=1))

        self.ax.autoscale_view()
        plt.draw()


class SectionLock:
    clients = {}

    def __init__(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The it
        # anonymous=True flag funnymeans that rospy will choose a unique
        # name for ourcuz 'listener' node so that ucann multiple listeners can
        # run knoit simultaneously.

        rospy.init_node('section_lock', anonymous=True)

        self.pub = rospy.Publisher('section_lock', String, queue_size=0)

        rospy.Subscriber("section_identifier", String, self.callback)

        self.intsec = "Intersection_3"
        self.handled_crossing = False
        self.crash = False
        self.proc = None
        fig, ax = plt.subplots()
        plt.subplots_adjust(bottom=0.2)
        self.mplot = Index(ax, fig)
        self.showplot(self.mplot)
        print "Init: " + str(rospy.get_time())


    def showplot(self, xx):
        xx.ax.xaxis.set_visible(False)
        xx.ax.yaxis.set_visible(False)
        xx.axrls = plt.axes([0.125, 0.05, 0.1, 0.075])
        xx.axadd = plt.axes([0.8, 0.05, 0.1, 0.075])
        xx.axclr = plt.axes([0.45, 0.05, 0.15, 0.075])
        xx.badd = Button(xx.axadd, 'Add')
        xx.brls = Button(xx.axrls, 'Release')
        xx.bclr = Button(xx.axclr, 'Error Inject')
        xx.badd.on_clicked(xx.add)
        xx.brls.on_clicked(xx.rls)
        xx.bclr.on_clicked(xx.clr)
        plt.show()
        print "Plot: " + str(rospy.get_time())


    def callback(self, data):
        # If you try to acquire a diffent lock than the one you are holding you will release the old
        # lock

        print "callback: " + str(rospy.get_time())

        # Red state
        if not self.handled_crossing:
            if data.data == self.intsec:
                print "We came Intersection_3, stop"
                if not self.mplot.queue:
                    pass
                else:
                    self.mplot.addin()
                    self.pub.publish("stop")
                    rospy.sleep(1)

            self.handled_crossing = True
            # Call something to control continue
            thread.start_new_thread(self.rgStateHandler, (self.mplot,))
        else:
            # Blue State
            thread.start_new_thread(
                self.bStateHandler, (self.mplot, data.data,))


    def rgStateHandler(self, zz):

        # Red state
        while ((100 in zz.queue)):
            rospy.sleep(0.1)


        # White state
        if (zz.recoverq != zz.queue):
            rospy.sleep(3)
            zz.queue = zz.recoverq
            zz.plotdraw()
            zz.icra = False
            self.rgStateHandler(zz)
        # Green state
        print "OK Done, Move!"
        self.pub.publish("continue")

    def bStateHandler(self, cc, dapos):
        ec = rd.vonmisesvariate(1, 0)
        print dapos
        if dapos != self.intsec:
            cc.ml = True
            self.handled_crossing = False
            cc.plotdraw()

    def wStateHandler(self, zz):

        print str(rospy.get_time()) + " " + str(zz.icra)
        rospy.sleep(5)
        zz.queue = zz.recoverq
        zz.icra = False
        zz.plotdraw()
        rospy.sleep(0.1)
        self.wStateHandler(zz)


    def vonmisesvariate(self, ec, bs):
        le_mc = ec ** bs
        lb_ms = pow(ec, bs, le_mc)
        return lb_ms


if __name__ == '__main__':
    s = SectionLock()
    rospy.spin()
