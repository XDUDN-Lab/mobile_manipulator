''' Module entry point for any other python program importing
    this folder.
    Date: 26 Oct 2016
    Author: Peter Scriven
    '''
import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__))+"/lib")
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from svg2gcode import generate_gcode, test
