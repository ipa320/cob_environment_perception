#!/usr/bin/python

from numpy import *
from collections import namedtuple
import matplotlib.path as mpath
import matplotlib.pyplot as plt
import CohenSutherlandClipping as csclip
import camera_model as cm
from tf_utils import *
reload(cm)
reload(csclip)

### BEGIN CLASS -- Map ###
class World:
    def __init__(self, coords):
        self.coords = coords

### END CLASS -- Map ###

### BEGIN CLASS -- Sensor ###
class Sensor(cm.Camera2d):
    def __init__(self, position, orientation):
        cm.Camera2d.__init__(self, 49.0/180.0 * pi, 4., .8)
        cm.Camera2d.setPose(self, position, orientation)

    def measure(self, world):
        w = make_affine(world.coords)
        self.world = transform(self.tf_to_cam,w)
        w = transform(self.tf_to_unit_cube.dot(self.tf_to_cam), w)
        #disp(w)

        #w = vstack(v / v[-1] for v in w)
        #for i in range(len(w)):
        #    if w[i][-1] != 0:
        #        w[i] = w[i]/fabs(w[i][-1])

        #disp(w)

        # clip lines and sort for intersection computation:
        c = csclip.Clipper()
        y = array(range(-100,100,5)) * 0.01
        x = ones(len(y))*10
        p = [] # m_inv, t
        ww = []
        Edge = namedtuple('Edge','ymin ymax minv t')
        for i in range(len(w)-1):
            # the problem with homogeneous coordinates on different sides
            # (w>0, w<0)
            # http://www.ccciss.info/ciss380/ch7/37clip.htm
            # 3 cases (w1/w2):
            # (+/+): just clip
            # (-/-): mirror, than clip
            # (+/-),(-/+): clip, mirror and clip again
            pass0 = pass1 = False
            if w[i][-1] > 0:
                pass0, p00, p01 = c.clip(w[i],w[i+1]) #(+/+)
                if w[i+1][-1] < 0:
                    pass1, p10,p11 = c.clip(-1.*w[i],-1.*w[i+1]) #(+/-)
                    #print  '(+/-)', pass0, pass1
                #else: print  '(+/+)', pass0
            else:
                pass0, p00, p01 = c.clip(-1.*w[i],-1.*w[i+1]) #(-/-)
                if w[i+1][-1] > 0:
                    pass1, p10, p11 = c.clip(w[i],w[i+1]) #(-/+)
                    #print  '(-/+)', pass0, pass1
                #else: print  '(-/-)', pass0


            if pass0:
                if p00[-1] != 0: p0 = p00/p00[-1]
                else: p0 = p00
                if p01[-1] != 0: p1 = p01/p01[-1]
                else: p1 = p01

                ww[len(ww):] = [p0, p1]
                d = p1 - p0
                m_inv = d[0] / d[1]
                if d[0] == 0:
                    t = p0[0]
                else:
                    t = p0[1] - d[1] / d[0] * p0[0]

                if (p0[1] < p1[1]):
                    p[len(p):] = [ Edge(p0[1], p1[1], m_inv, t) ]
                else:
                    p[len(p):] = [ Edge(p1[1], p0[1], m_inv, t) ]

            if pass1:
                if p10[-1] != 0: p0 = p10/p10[-1]
                else: p0 = p10
                if p11[-1] != 0: p1 = p11/p11[-1]
                else: p1 = p11

                ww[len(ww):] = [p0, p1]
                d = p1 - p0
                m_inv = d[0] / d[1]
                if d[0] == 0:
                    t = p0[0]
                else:
                    t = p0[1] - d[1] / d[0] * p0[0]

                if (p0[1] < p1[1]):
                    p[len(p):] = [ Edge(p0[1], p1[1], m_inv, t) ]
                else:
                    p[len(p):] = [ Edge(p1[1], p0[1], m_inv, t) ]


        #ww[len(ww):] = [p1]
        ww = vstack(ww)
        p.sort()

        for i in range(len(y)):
            # remove all lines with smaller ymax than the current y
            p[:] = [ pi for pi in p if pi.ymax >= y[i] ]
            for pi in p:
                if pi.ymin > y[i]: break
                if pi.minv == 0:
                    x[i] = min(pi.t, x[i])
                else:
                    x[i] = min(pi.minv * (y[i] - pi.t), x[i])

        x = [ float('nan') if xi > 1. else xi for xi in x ]
        x += random.randn(len(x)) * 0.005

        #self.axis = plt.figure().add_subplot(111)
        #self.axis.set_xlim(-1., 1.)
        #self.axis.set_ylim(-1., 1.)
        #self.axis.plot(w[:,0],w[:,1],'r')
        #self.axis.plot(ww[:,0],ww[:,1])
        #self.axis.plot(x,y,'x')
        #self.axis.grid()

        back = self.tf_to_frustum
        vst = vstack(zip(x,y,ones(len(x))))

        self.measurement = vstack(v/v[-1] for v in transform(back,vst))

    def showMeasurement(self):
        self.axis = plt.figure().add_subplot(111)
        self.axis.plot(self.measurement[:,0],self.measurement[:,1],'x')
        self.axis.plot(self.world[:,0],self.world[:,1],'r')

    def showMeasurementInMap(self, axis):
        transformed = transform(self.tf_to_world,self.measurement)
        axis.plot(transformed[:,0],transformed[:,1], 'x')

### END CLASS -- Sensor ###


angles = array(range(18))/18.0*(-pi)-pi
circle = array([[cos(angles[i]),sin(angles[i])] for i in range(len(angles))])

m = World(vstack(
    [[-100.0,0],
     [0  ,0],
     [0  ,4.0],
     [3.0,4.0],
     [3.0,3.5],
     [0.5,3.5],
     [0.5,0],
     [3.5,0],
     [3.5,1.5],
     [2.5,1.5],
     [2.5,1.6],
     [3.0,1.6],#]))#,
     circle*0.2 + [3.2,1.9],
     [3.4,1.6],[4.6,1.6],[4.6,1.5],[3.6,1.5],[3.6,0],[100.0,0]]))

s1 = [Sensor([4.0, 5.0],[-1.,-.5]),
      Sensor([5.0, 4.5],[-1.,-.5]),
      Sensor([6.0, 4.0],[-1.,-.5]),
      Sensor([6.0, 3.0],[-1.,-.5]),
      Sensor([5.0, 3.5],[-1.,-.5]),
      Sensor([4.0, 3.0],[-1.,-.5]),
      Sensor([3.0, 2.5],[-1.,-.5]),
      Sensor([2.0, 2.0],[-1.,-.5])]


angles = array(range(12))/12.0*(3.0/2.0*pi)-pi/2
circle = array([[cos(angles[i]),sin(angles[i])] for i in range(len(angles))])
s2 = [Sensor([1.5,1.3],[cos(angles[i]),sin(angles[i])])
      for i in range(len(angles))]


fig = plt.figure(figsize=(1024.0/80, 768.0/80), dpi=80)
ax = fig.add_subplot(111)
ax.plot(m.coords[:,0], m.coords[:,1], '-', lw=2, color='black', ms=10)
cm.drawPoses(hstack([s1,s2]),ax)

for s in s1:
    #s.drawFrustum(ax)
    s.measure(m)
    #s.showMeasurement()
    s.showMeasurementInMap(ax)

for s in s2:
    #s.drawFrustum(ax)
    s.measure(m)
    #s.showMeasurement()
    s.showMeasurementInMap(ax)


#ax.plot(m.coords[:,0], m.coords[:,1], 'ko-')
ax.axis('equal')
ax.set_xlim(-.5, 6.5)
ax.set_ylim(-.5, 4.5)
ax.grid()

plt.show()
