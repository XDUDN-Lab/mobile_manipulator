# -*- coding: utf-8 -*-
import cv2
import sys
import os
import numpy as np
import argparse
import py_svg2gcode

if __name__=="__main__":
    parser = argparse.ArgumentParser("Convert svg to G-code")
    parser.add_argument('filename', help="SVG file path")
    args = parser.parse_args()
    py_svg2gcode.generate_gcode(args.filename)