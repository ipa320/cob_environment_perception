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

''' Edges are sorted by ymin.
s= 1: front face
s=-1: back face
'''
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

class ScanlineRasterization:
    def __init__(self):
        self.e = []

    def addEdge(self, p1, p2):
        self.e[len(self.e):] = [ Edge(p1,p2) ]

    def draw(self, limits = [-1.,1.,-1.,1.], cellsize = [0.05,0.05]):
        xmin = limits[0]
        xmax = limits[1]
        xstep = cellsize[0]
        ymin = limits[2]
        ymax = limits[3]
        ystep = cellsize[1]
        y = [ i*ystep for i in range(int(ymin/ystep), int(ymax/ystep)+1) ]
        x = [ xmax for i in range(len(y)) ]

        self.e.sort() # sort edges by ymin
        for i in range(len(y)): # start with lowest y value
            # remove lines with ymax < current y
            p = [ pi for pi in self.e if pi.ymax >= y[i] ]
            for pi in p:
                # stop as soon as lines start above current y
                if pi.ymin > y[i]: break
                x[i] = min(pi.intersect(y[i]),x[i])

        return x,y

    def fill(self, limits = [-1.,1.,-1.,1.], cellsize = [0.05,0.05]):
        xmin = limits[0]
        xmax = limits[1]
        xstep = cellsize[0]
        ymin = limits[2]
        ymax = limits[3]
        ystep = cellsize[1]
        y = [ i*ystep for i in range(int(ymin/ystep), int(ymax/ystep)+1) ]
        x = [ i*xstep for i in range(int(xmin/xstep), int(xmax/xstep)+1) ]
        grid = zeros([len(y), len(x)])

        self.e.sort() # sort edges by ymin
        for yi in range(len(y)): # iterate lines starting with lowest y value
            p = [ pi for pi in self.e if pi.ymax >= y[yi] ]
            yx = []
            for pi in p:
                # calc all intersection at current line and the direction how
                # the ray passes an edge
                if pi.ymin > y[yi]: break
                intx = pi.intersect(y[yi])
                if intx >= xmin:
                    yx[len(yx):] = [ (intx, pi.s) ]

            if len(yx) == 0:
                for xi in range(len(x)):
                    grid[yi][xi] = 0
                continue
            yx.sort()
            yxi = 0
            if yx[yxi][1] < 0:
                level = 1
            else:
                level = 0

            for xi in range(len(x)):
                if x[xi] <= yx[yxi][0]:
                    grid[yi][xi] = level
                else:
                    level = level + yx[yxi][1]
                    yxi = yxi+1
                    if yxi >= len(yx): break

        return grid
