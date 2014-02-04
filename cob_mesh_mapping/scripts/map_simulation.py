#!/usr/bin/python

#%load_ext autoreload
#%autoreload 2

from numpy import *
from collections import namedtuple
import matplotlib.path as mpath
import matplotlib.pyplot as plt
import CohenSutherlandClipping as csclip
import camera_model as cm
from tf_utils import *
from normal_estimation import *
from mesh_structure import *
from mesh_optimization import *
import measurement_data as mdata
import scanline_rasterization as sl
import iterative_mesh_learner as iml


### BEGIN CLASS -- Map ###
class World:
    def __init__(self, coords):
        self.coords = coords

    def draw(self, axis):
        axis.plot(self.coords[:,0], self.coords[:,1],
                  '-', lw=2, alpha=0.2, color='black', ms=10)

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
        ww = []
        scan = sl.ScanlineRasterization()

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
            else:
                pass0, p00, p01 = c.clip(-1.*w[i],-1.*w[i+1]) #(-/-)
                if w[i+1][-1] > 0:
                    pass1, p10, p11 = c.clip(w[i],w[i+1]) #(-/+)

            if pass0:
                if p00[-1] != 0: p0 = p00/p00[-1]
                else: p0 = p00
                if p01[-1] != 0: p1 = p01/p01[-1]
                else: p1 = p01

                ww[len(ww):] = [p0, p1]
                scan.addEdge(p0,p1)

            if pass1:
                if p10[-1] != 0: p0 = p10/p10[-1]
                else: p0 = p10
                if p11[-1] != 0: p1 = p11/p11[-1]
                else: p1 = p11

                ww[len(ww):] = [p0, p1]
                scan.addEdge(p0,p1)

        ww = vstack(ww)
        x,y = scan.draw([-1.,1.,-1.,1.], [2./self.res,2./self.res])
        x = [ float('nan') if xi >= 1. else xi for xi in x ]
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



###----------------------------------------------------------------------------
#     provide simulation data (world, sensors, measurements)
###----------------------------------------------------------------------------

# create world model:
angles = array(range(18))/18.0*(-pi)-pi
circle = array([[cos(angles[i]),sin(angles[i])] for i in range(len(angles))])

world = World(vstack(
    [[-100.0,0],[0,0],[0,4.0],[3.0,4.0],[3.0,3.5],[0.5,3.5],
     [0.5,0],[3.5,0],[3.5,1.5],[2.5,1.5],[2.5,1.6],[3.0,1.6],#]))#,
     circle*0.2 + [3.2,1.9],
     [3.4,1.6],[4.6,1.6],[4.6,1.5],[3.6,1.5],[3.6,0],[100.0,0]]))

# create sensors and measure world:
s1 = [Sensor([4.0, 5.0],[-1.,-.5]),
      Sensor([5.0, 4.5],[-1.,-.5]),
      Sensor([6.0, 4.0],[-1.,-.5]),
      Sensor([6.0, 3.0],[-1.,-.5]),
      Sensor([5.0, 3.5],[-1.,-.5]),
      Sensor([4.0, 3.0],[-1.,-.5]),
      Sensor([3.0, 2.5],[-1.,-.5]),
      Sensor([2.0, 2.0],[-1.,-.5])]

circle_size = 12.0
angles = array(range(int(circle_size)))/circle_size*(3.0/2.0*pi)-pi/2
circle = array([[cos(angles[i]),sin(angles[i])] for i in range(len(angles))])
s2 = [Sensor([1.5,1.3],[cos(angles[i]),sin(angles[i])])
      for i in range(len(angles))]

sensors = hstack([s1,s2])
for s in sensors: s.measure(world)

###----------------------------------------------------------------------------
#     process measurements and add to map
###----------------------------------------------------------------------------

learner = iml.IterativeMeshLearner()
data = []
colors = 'ym'
iii = 0
sensors = [sensors[3], sensors[6], sensors[12]]
for s in sensors:
    # normal estimation:
    ii = len(s.measurement[:,0]) # number of measurement points
    ne = Normals2d(9, ii)
    n = empty([ii,2])
    for i in range(ii):
        n[i] = ne.computeNormal(s.measurement[:,0],s.measurement[:,1],i)

    # create mesh:
    m = Mesh()
    m.load(s.measurement[:,0],s.measurement[:,1],n[:,0],n[:,1])
    #s.axis = plt.figure().add_subplot(111)
    #m.draw(s.axis,'ven')

    # simplify mesh:
    ms = Simplifier()
    ms.init(m)
    ms.simplify(1.0)

    #m.draw(s.axis,'ve', 'kr'+colors[iii%len(colors)])

    # convert simplified mesh to input format for map optimization:
    data_new = mdata.convertMeshToMeasurementData(m,s)
    learner.initMesh(data_new[0])
    learner.addMeasurement(data_new[1])
    learner.addMeasurement(data_new[2])
    data[len(data):] = [data_new[0],data_new[1],data_new[2]]
    iii = iii+1



###----------------------------------------------------------------------------
#     visualize results
###----------------------------------------------------------------------------

fig1 = plt.figure(figsize=(1024.0/80, 768.0/80), dpi=80)
fig2 = plt.figure(figsize=(1024.0/80, 768.0/80), dpi=80)
ax1 = fig1.add_subplot(111)
ax2 = fig2.add_subplot(111)

world.draw(ax1)
cm.drawPoses(sensors,ax1)
cm.drawPoses(sensors,ax2)
for s in sensors: s.showMeasurementInMap(ax1)
#for s in sensors: s.drawFrustum(ax1)
for d in data:
    d.draw(ax2)
    d.drawBoundingBox(ax2,[.1,.1])

ax1.axis('equal')
ax1.set_xlim(-.5, 6.5)
ax1.set_ylim(-.5, 4.5)
ax1.grid()

ax2.axis('equal')
ax2.set_xlim(-.5, 6.5)
ax2.set_ylim(-.5, 4.5)
ax2.grid()

plt.show()
