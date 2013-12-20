#!/usr/bin/python
from numpy import *
import heapq
from collections import namedtuple
import mesh_structure as ms
import matplotlib.pyplot as plt
reload(ms)

class Heap:
    def __init__(self):
        self.h = []

    def push(self, cost, data, op):
        Item = namedtuple('Item', 'cost, data, op')
        heapq.heappush(self.h,Item(cost,data,op))

    def pop(self):
        return heapq.heappop(self.h)

class Simplifier:
    def __init__(self, mesh = None):
        """vertices of mesh require normal information beforhand"""
        self.heap = Heap()
        if mesh is not None:
            self.init(mesh)

    def init(self, mesh):
        self.mesh = mesh
        for e in mesh.E:
            c = self.compute_cost(e, 'EC')
            self.heap.push(c,e,'EC')

    def compute_cost(self, data, operation):
        """
        data: edge or vertex object
        operation: [EC,ES,VM]
        """
        if operation is 'EC':
            Q = data.v1.Q + data.v2.Q
            q = array(vstack([ Q[0:2,:], [0,0,1.] ]))
            #A = Q[:2,:2]
            #B = Q[2,:2]
            #C = Q[2,2]
            det = fabs(linalg.det(q))
            if det > 0.0001:
                v = linalg.inv(q).dot(array([[0],[0],[1.]]))
                #v = linalg.inv(A).dot(B)
            else:
                v = array([[0.5 * (data.v1.x + data.v2.x)],
                           [0.5 * (data.v1.y + data.v2.y)],[1]])

            data.vnew = ms.Vertex(float(v[0]),float(v[1]),Q)
            #cost = float(v.T.dot(A).dot(v) + 2.*v.T.dot(B) + C)
            cost = fabs(float(v.T.dot(Q).dot(v)))
            print det, cost

        elif operation is 'ES':
            pass

        elif operation is 'VM':
            pass

        data.dirty = False
        return cost

    def simplify(self, eps = 0.1, min_vertices = 6):
        cost = 0.0
        while(cost < eps ):#and len(self.mesh.V) > min_vertices):
            h = self.heap.pop()
            if h.data.dirty:
                c = self.compute_cost(h.data, h.op)
                self.heap.push(c,h.data,h.op)
            elif h.op is 'EC':
                self.mesh.collapse(h.data)
                cost = h.cost
            elif h.op is 'ES':
                self.mesh.split(h.data)
                cost = h.cost
            elif h.op is 'VM':
                seld.mesh.move(h.data)
                cost = h.cost


m = ms.Mesh()
m.test_load()
m.test_show()
s = Simplifier()
s.init(m)
s.simplify()
m.test_show()
plt.show()
