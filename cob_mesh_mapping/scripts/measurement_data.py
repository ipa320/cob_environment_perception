#!/usr/bin/python

from numpy import *

from camera_model import *
import mesh_structure

def convertMeshToMeasurementData(mesh, cam):
    res = []
    for e in mesh.E:
        #print e.v1.x, e.v1.y, " | ", e.v2.x, e.v2.y
        if math.isnan(e.v1.x) or math.isnan(e.v2.x): continue
        m1 = cam.tf_to_world.dot(array([e.v1.x, e.v1.y, 1.0]))
        m2 = cam.tf_to_world.dot(array([e.v2.x, e.v2.y, 1.0]))
        res[len(res):] = [MeasurementData(cam.pos,cam.ori,m1,m2)]
    return res

class MeasurementData(Camera2d):
    '''
    Measurement is counter-clock-wise:\n
    p: sensor position
    o: sensor orientation
    m1: measurement point 1
    m2: measurement point 2
    '''
    def __init__(self, p, o, m1, m2): #orientiation is wrong here!
        self.m1 = m1
        self.m2 = m2
        lm1 = linalg.norm(m1)
        lm2 = linalg.norm(m2)
        fov = math.acos(m2.T.dot(m1)/(lm1*lm2))
        f = math.cos(0.5*fov)*max(lm1,lm2)
        Camera2d.__init__(self, fov, f, 0.4)
        Camera2d.setPose(self, p, o)

    def draw(self, axis):
        axis.plot([self.m1[0], self.m2[0]], [self.m1[1], self.m2[1]], 'bx-')
