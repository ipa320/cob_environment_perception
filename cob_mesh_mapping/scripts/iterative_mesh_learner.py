#!/usr/bin/python

from numpy import *
import matplotlib.pyplot as plt
from mesh_structure import *
from measurement_data import *

class IterativeMeshLearner:
    def __init__(self):
        self.data = []

    def initMesh(self, measurement):
        self.mesh = Mesh()
        v0 = self.mesh.add(measurement.pos[0],measurement.pos[1])
        v1 = self.mesh.add(measurement.m1[0],measurement.m1[1])
        v2 = self.mesh.add(measurement.m2[0],measurement.m2[1])



    def addMeasurement(self, m):
        self.data[len(self.data):] = [m]
