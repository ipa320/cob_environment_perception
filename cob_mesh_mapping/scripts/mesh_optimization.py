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
            w = data.v1.w + data.v2.w
            Q = data.v1.Q + data.v2.Q
            Qw = Q / w
            q = array(vstack([ Qw[0:2,:], [0,0,1.] ]))
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

            data.vnew = ms.Vertex(float(v[0]),float(v[1]),Q,w)
            #cost = float(v.T.dot(A).dot(v) + 2.*v.T.dot(B) + C)
            cost = fabs(float(v.T.dot(Qw).dot(v)))
            #cost = fabs(float(v.T.dot(Q).dot(v)))
            #print det, cost

        elif operation is 'ES':
            pass

        elif operation is 'VM':
            pass

        data.dirty = False
        return cost

    def simplify(self, eps = 0.1, min_vertices = 3):
        h = self.heap.pop()
        while(h.cost < eps and len(self.mesh.V) > min_vertices):
            if h.op is 'EC':
                #print h.cost, h.data.v1.w, h.data.v2.w
                self.mesh.collapse(h.data)
            elif h.op is 'ES':
                self.mesh.split(h.data)
            elif h.op is 'VM':
                seld.mesh.move(h.data)

            h = self.heap.pop()
            while(h.data.dirty):
                c = self.compute_cost(h.data, h.op)
                self.heap.push(c,h.data,h.op)
                h = self.heap.pop()


#m = ms.Mesh()
#m.test_load()
#m.test_show()
#s = Simplifier()
#s.init(m)
#s.simplify(10.0,5)
#m.test_show()
#plt.show()
