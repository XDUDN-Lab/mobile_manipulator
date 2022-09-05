#!/usr/bin/env python2
from pygcode import *
import os
import gcode_excute
import arm_controller
import rospy
import geometry_msgs.msg
import sys
import argparse
import signal

def signal_handler(signal, frame):
    sys.exit()
class Interpreter():
    def __init__(self):
        self.file = None
        self.mac = Machine()
        self.planner = None
        self.fr = 0
        self.EOF = False
        self.mac.ignore_invalid_modal = True

    def setPlanner(self, planner):
        self.planner = planner

    def loadFile(self, fname):
        if not (self.file is None):
            self.file.close()
        self.file = open(fname, "r")

    def gcode_draw(self,fname):
        self.loadFile(fname)
        for line_txt in self.file.readlines():
            line = Line(line_txt)
            if len(line.block.gcodes) == 0:
                continue
            
            block = self.mac.block_modal_gcodes(line.block)
            xb, yb, zb = self.mac.pos.vector
            self.mac.process_gcodes(*sorted(block))
            xa, ya, za = self.mac.pos.vector
            for b in block:
                if isinstance(b, GCodeLinearMove):
                    self.planner.G1(self.mac.pos.vector[0], self.mac.pos.vector[1], self.fr)
                    # return True
                elif isinstance(b, GCodeRapidMove):
                    self.planner.G0(self.mac.pos.vector[0], self.mac.pos.vector[1])
                    # return True
                elif isinstance(b, GCodeFeedRate):
                    self.fr = b.word.value
                elif isinstance(b, GCodeArcMoveCW):
                    # I don't know how any of this works, just copied from a pygcode arc linearizing example
                    arc_center_ijk = dict((l, 0.) for l in "IJK")
                    arc_center_ijk.update(b.get_param_dict("IJK"))
                    arc_center_coords = dict(({"I":"x", "J":"y", "K":"z"}[k], v) for (k, v) in arc_center_ijk.items())
                    # incremental position
                    xc = arc_center_coords['x'] + xb
                    yc = arc_center_coords['y'] + yb
                    self.planner.G2(self.mac.pos.vector[0], self.mac.pos.vector[1], xc ,yc, self.fr)
                    # return True
                elif isinstance(b, GCodeArcMoveCCW):
                    arc_center_ijk = dict((l, 0.) for l in "IJK")
                    arc_center_ijk.update(b.get_param_dict("IJK"))
                    arc_center_coords = dict(({"I":"x", "J":"y", "K":"z"}[k], v) for (k, v) in arc_center_ijk.items())
                    xc = arc_center_coords['x'] + xb
                    yc = arc_center_coords['y'] + yb
                    self.planner.G3(self.mac.pos.vector[0], self.mac.pos.vector[1], xc, yc, self.fr)
                    # return True
                elif isinstance(b,GCodeStopSpindle):
                    print "Pen stop"
                    self.planner.M5()
                elif isinstance(b,GCodeStartSpindle):
                    print "Pen start"
                    self.planner.M3()
                else:
                    print "Unknown Instruction: ", b
            # return False

if __name__=="__main__":
    rospy.init_node("gcode_draw_core",anonymous=True)
    # capture SIGINT signal, e.g., Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGQUIT, signal_handler)
    parser = argparse.ArgumentParser("Draw gcode with moveit!")
    parser.add_argument('filename', help="Gcode file path")
    args, unpar = parser.parse_known_args()
    g = Interpreter()
    arm_draw = arm_controller.Arm_Contrl()
    arm_draw.go_home()
    now_pose = arm_draw.get_current_pos()

    arm_draw.up_pen_height = now_pose.position.z + 0.05
    arm_draw.down_pen_height = now_pose.position.z
    print("ready", now_pose.position.x, now_pose.position.y)
    manipulator = gcode_excute.gcode_excute(now_pose.position.x,now_pose.position.y,arm_draw)

    g.setPlanner(manipulator)

    print('--->Load file:',args.filename )
    g.gcode_draw(args.filename)

