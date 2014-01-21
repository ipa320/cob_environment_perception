#!/usr/bin/python

from numpy import *
import camera_model as cm

class Measurement(cm.Camera2d):
    '''
    Measurement is counter-clock-wise:\n
    p: sensor position
    m1: measurement point 1
    m2: measurement point 2
    '''
    def __init__(self, p, m1, m2):
        lm1 = linalg.norm(m1)
        lm2 = linalg.norm(m2)
        fov = math.arccos(m2.T.dot(m1)/(lm1*ml2))
        f = math.cos(0.5*fov)*max(lm1,lm2)
        cm.Camera2d.__init__(self, fov, f, 0.4)
        cm.Camera2d.setPose(self, position, orientation)
