#!/usr/bin/python

from numpy import *

# if (y2 - y1) > 0 front face
# or: the rotation(+90deg) of vector (p2 - p1) always points
# towards the sensor position

class Intersection:
    def __init__(self, minv, t):
        self.minv = minv
        self.t = t

    def calc(self,y):
        return self.minv*(y-self.t)

class IntersectionSimple:
    def __init__(self, t):
        self.t = t

    def calc(self,y):
        return self.t

class Edge:
    '''
    p1: bottom
    p2: top
    '''
    def __init__(self, p1, p2):
        d = p2 - p1

        if d[1] > 0:
            self.ymin = float(p1[1])
            self.ymax = float(p2[1])
            self.s = 1
        else:
            self.ymin = float(p2[1])
            self.ymax = float(p1[1])
            self.s = -1

        if d[0] == 0:
            #self.t = p1[0]
            #self.minv = 0
            self.intersection = IntersectionSimple(float(p1[0]))
        elif d[1] == 0:
            #self.t = float('nan')
            #self.minv = 0
            self.intersection = IntersectionSimple(float('nan'))
        else:
            #self.t = p1[1]-(d[1]/d[0]*p1[0])
            #self.minv = d[0]/d[1]
            self.intersection = Intersection(float(d[0]/d[1]),
                                             float(p1[1]-(d[1]/d[0]*p1[0])))


    def __cmp__(self,other):
        return cmp(self.ymin,other.ymin)

    def intersect(self, y):
        #if self.minv==0: return self.t
        #else: return self.minv*(y-self.t)
        return self.intersection.calc(y)

class ScanlineFill:
    def __init__(self):
        self.e = []


