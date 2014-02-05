#!/usr/bin/python

from numpy import *
import matplotlib.pyplot as plt
from mesh_structure import *
import CohenSutherlandClipping as csclip
import scanline_rasterization as sl
from tf_utils import *

class IterativeMeshLearner:
    def __init__(self):
        self.data = []
        self.mesh = Mesh()

    def initMesh(self, measurement):
        v1 = self.mesh.add(measurement.m1[0],measurement.m1[1])
        v2 = self.mesh.add(measurement.m2[0],measurement.m2[1])
        self.mesh.connect(v1,v2)
        self.data.append(measurement)

    def addMeasurement(self, m):
        self.data.append(m)
        scan = sl.ScanlineRasterization()
        scan.addEdge(m.m1,m.m2)
        for e in self.mesh.E:
            #print e
            scan.addEdge(e.v1.getPos(),e.v2.getPos())

        # rasterize bounding box
        lim = m.getBoundingBox([.1,.1])
        grid = scan.fill(lim, [.05,.05])
        # marching cubes mesh reconstruction

    def refineMesh(self, data, cam):
        # first create virtual sensor at current position
        # and reconstruct measurements based on current map
        # however: remember anchor vertices of map where
        # the refined mesh is going to be hooked up on
        v_hooks = []
        if len(self.mesh.V) != 0:
            v_hooks = [(100.0, self.mesh.V[0]), # [0]: y<0 (bottom),
                       (100.0, self.mesh.V[0])] # [1]: y>0 (top)
        else:
            v_hooks = [(100.0, None),(100.0, None)]

        c = csclip.Clipper()
        scan = sl.ScanlineRasterization()
        tf = cam.tf_to_unit_cube.dot(cam.tf_to_cam)
        for e in self.mesh.E:
            w0 = tf.dot(e.v1.getPosAffine()) # redundant transform
            w1 = tf.dot(e.v2.getPosAffine()) # TODO: do better!

            pass0 = pass1 = False
            if w0[-1] > 0:
                pass0, p00, p01 = c.clip(w0,w1) #(+/+)
                if w1[-1] < 0:
                    pass1, p10,p11 = c.clip(-1.*w0,-1.*w1) #(+/-)
            else:
                pass0, p00, p01 = c.clip(-1.*w0,-1.*w1) #(-/-)
                if w1[-1] > 0:
                    pass1, p10, p11 = c.clip(w0,w1) #(-/+)

            if pass0:
                if p00[-1] != 0: p0 = p00/p00[-1]
                else: p0 = p00
                if p01[-1] != 0: p1 = p01/p01[-1]
                else: p1 = p01

                if w0 is not p00 and w0 is not p01: # w0 got clipped
                    # keep closest (smallest x) outer vertex
                    idx = int(p0[1]>0)
                    if p0[0] < v_hooks[idx][0]:
                        v_hooks[idx] = ( (p0[0], e.v1) )
                if w1 is not p00 and w1 is not p01: # w1 got clipped
                    # keep closest (smallest x) outer vertex
                    idx = int(p0[1]>0)
                    if p0[0] < v_hooks[idx][0]:
                        v_hooks[idx] = ( (p0[0], e.v2) )

                scan.addEdge(p0,p1)
                e.dirty = True # mark for deletion

            if pass1:
                if p10[-1] != 0: p0 = p10/p10[-1]
                else: p0 = p10
                if p11[-1] != 0: p1 = p11/p11[-1]
                else: p1 = p11

                if w0 is not p10 and w0 is not p11: # w0 got clipped
                    # keep closest (smallest x) outer vertex
                    idx = int(p1[1]>0)
                    if p1[0] < v_hooks[idx][0]:
                        v_hooks[idx] = ( (p1[0], e.v1) )
                if w1 is not p10 and w1 is not p11: # w1 got clipped
                    # keep closest (smallest x) outer vertex
                    idx = int(p1[1]>0)
                    if p1[0] < v_hooks[idx][0]:
                        v_hooks[idx] = ( (p1[0], e.v2) )

                scan.addEdge(p0,p1)
                e.dirty = True # mark for deletion
        # END: for e in self.mesh.E:

        self.mesh.cleanup()
        x,y = scan.contour([-1.,1.,-1.,1.], [2./cam.res,2./cam.res])
        vst = vstack(zip(x,y,ones(len(x))))
        m = vstack(v/v[-1] for v in transform(cam.tf_to_frustum,vst))

        # second: combine real measurement data with virtually generated
        # and insert in existing mesh
        ii = len(data[:,0])
        for i in range(ii):
            if m[i][0] > data[i][0]:
                m[i] = data[i]
            elif m[i][0] >= (cam.f):
                m[i][0] = float('nan')


        #print m
        #ax = plt.figure().add_subplot(111)
        #ax.axis('equal')
        #ax.plot(data[:,0],data[:,1],'xr')
        #ax.plot(m[:,0],m[:,1],'xb')

        m = transform(cam.tf_to_world,m)

        if v_hooks[0][0] < 100.0:
            v1 = v_hooks[0][1]
            i = 0
        else:
            for i in range(ii):
                if not math.isnan(m[i][0]): break
            v1 = self.mesh.add(m[i][0],m[i][1])

        for j in range(i+1,ii):
            if math.isnan(m[j][0]): continue

            v2 = self.mesh.add(m[j][0],m[j][1])
            self.mesh.connect(v1,v2)
            v1 = v2

        if v_hooks[1][0] < 100.0:
            v2 = v_hooks[1][1]
            self.mesh.connect(v1,v2)

        #print "MESH.V:"
        #print self.mesh.V
