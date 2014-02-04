#!/usr/bin/python

from numpy import *
import matplotlib.pyplot as plt
from mesh_structure import *
from measurement_data import *
import scanline_rasterization as sl

class IterativeMeshLearner:
    def __init__(self):
        self.data = []

    def initMesh(self, measurement):
        self.mesh = Mesh()
        v1 = self.mesh.add(measurement.m1[0],measurement.m1[1])
        v2 = self.mesh.add(measurement.m2[0],measurement.m2[1])
        self.mesh.connect(v1,v2)
        self.data[len(self.data):]= [measurement]


    def addMeasurement(self, m):
        self.data[len(self.data):] = [m]
        scan = sl.ScanlineRasterization()
        scan.addEdge(m.m1,m.m2)
        for e in self.mesh.E:
            #print e
            scan.addEdge(e.v1.getPos(),e.v2.getPos())

        # rasterize bounding box
        lim = m.getBoundingBox([.1,.1])
        grid = scan.fill(lim, [.05,.05])
        # marching cubes mesh reconstruction
