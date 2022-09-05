# -*- coding: utf-8 -*-
#!/usr/bin/env python
import numpy as np
import arm_controller
import sys
import rospy
import time
import copy
import math

# msg stuff
import moveit_msgs.msg
import geometry_msgs.msg

class gcode_excute():
    def __init__(self, x0, y0,Controller):
        self.x0, self.y0 = x0, y0 # coordinate offset

        self.s = -0.001 # scale factor (mm -> m)

        # self.amax = 0.3

        # instanlize controller
        self.controller = Controller

    def G0(self, x, y): # the "just go there" instruction
        xn, yn = self.tf(x, y)
        print "G0",xn,yn
        # self.path = line.Line(self.x, self.y, xn, yn, 0.5, self.amax)
        target_point = self.controller.get_current_pos()
        target_point.position.x = xn
        target_point.position.y = yn
        
        self.controller.draw_line(target_point)

    def G1(self, x, y, fr): # move to x, y in a line at a speed in mm/minute
        fr /= 1000 * 60 # mm/minute -> m/sec
        xn, yn = self.tf(x, y)
        print "G1",xn,yn
        # self.path = line.Line(self.x, self.y, xn, yn, fr, self.amax)
        target_point = self.controller.get_current_pos()
        target_point.position.x = xn
        target_point.position.y = yn
        self.controller.draw_line(target_point)

    def M3(self):   # Down the pen
        self.controller.down_pen()
    
    def M5(self):   # Up the pen
        self.controller.up_pen()

    def setPosition(self, x, y):
        self.x, self.y = x, y

    def tf(self, x, y):     # Transform
        x = x * self.s + self.x0
        y = y * self.s + self.y0
        return (x, y)


    # This is function for high accurate draw circle. It's not necessary
    def G2(self, x, y, xc, yc, fr): # move in a clockwise arc to an endpoint around a center
        fr /= 1000 * 60 # mm/minute -> m/sec
        xn, yn = self.tf(x, y)
        xcn, ycn = self.tf(xc, yc)
        print "G2",xn,yn,xcn,ycn
        # self.path = arc.Arc(self.x, self.y, xn, yn, xcn, ycn, fr, self.amax, cw=True)
        
    def G3(self, x, y, xc, yc, fr):
        fr /= 1000 * 60
        xn, yn = self.tf(x, y)
        xcn, ycn = self.tf(xc, yc)
        print "G3",xn,yn,xcn,ycn
        # self.path = arc.Arc(self.x, self.y, xn, yn, xcn, ycn, fr, self.amax, cw=False)
    