from numpy import *
import heapq
from collections import namedtuple
import matplotlib.pyplot as plt
import normal_estimation
reload(normal_estimation)

class Vertex:
    def __init__(self, x, y, q = zeros([3,3])):
        """ """
        self.x = x
        self.y = y
        self.Q = q
        self.e1 = None
        self.e2 = None


    def addPlaneParam(self, nx, ny):
        """ """
        #self.nx = nx
        #self.ny = ny
        d = -(nx*self.x + ny*self.y)
        q = array([[nx],[ny],[d]])
        Q = q.dot(q.T)
        #a = math.degrees(math.atan2(ny, nx))
        #if a < 0: print a+180
        #else: print a
        self.Q = self.Q + Q

    def __repr__(self):
        return "(%3.2f %3.2f)" % (self.x, self.y)

class Edge:
    def __init__(self, v1, v2):
        """ """
        self.v1 = v1
        self.v2 = v2
        self.vnew = v1
        self.dirty = False

    def __repr__(self):
        return `self.v1.__repr__()` + "  <--->  "  + `self.v2.__repr__()`

class Mesh:
    def __init__(self):
        """ """
        self.V = []
        self.E = []

    def add(self,x,y):
        """ """
        v = Vertex(x,y)
        self.V.append(v)
        return v

    def connect(self,v1,v2):
        """ """
        e = Edge(v1,v2)
        v1.e2 = e
        v2.e1 = e
        self.E.append(e)
        return e

    def collapse(self, e):
        vnew = e.vnew
        if e.v1.e1 is not None:
            vnew.e1 = e.v1.e1
            vnew.e1.v2 = vnew
            vnew.e1.dirty = True
        if e.v2.e2 is not None:
            vnew.e2 = e.v2.e2
            vnew.e2.v1 = vnew
            vnew.e2.dirty = True

        self.V.append(vnew)
        self.V.remove(e.v1)
        self.V.remove(e.v2)
        self.E.remove(e)

    def split(self, e):
        pass

    def move(self, v):
        pass

    def draw(self, axis, options = 'ven'):
        """ options: n=normals, e=edges, v=vertices """
        if 'v' in options:
            x = [ v.x for v in self.V ]
            y = [ v.y for v in self.V ]
            axis.plot(x,y,'ok')

        if 'n' in options:
            for v in self.V:
                x = [v.x, v.x + v.nx]
                y = [v.y, v.y + v.ny]
                axis.plot(x,y,'r')

        if 'e' in options:
            for e in self.E:
                x = [e.v1.x, e.v2.x]
                y = [e.v1.y, e.v2.y]
                axis.plot(x,y,'b')

    def test_load(self):
        self.z = array([nan,         nan,         nan,  7.99356667,  7.82893667,
                   7.57826583,  7.24122648,  7.00702245,  6.80268191,  6.62213153,
                   6.39752627,  6.26145436,  6.0878814 ,  5.85069153,  5.75376865,
                   5.62025108,  5.4464248 ,  5.26246435,  5.20527609,  5.06965995,
                   4.93485073,  4.83511075,  4.74354981,  4.64631025,  4.53356859,
                   4.45845244,  4.36818577,  4.24801165,  4.19271887,  4.12449824,
                   4.01506922,  3.95092438,  3.90107743,  4.02811415,  4.13162295,
                   4.30167701,  4.45022342,  4.61539678,  4.77774431,  4.98734557])
        self.x = array([nan,         nan,         nan, -3.09644647, -2.85428159,
                   -2.59021103, -2.31001192, -2.07563467, -1.86009645, -1.65983356,
                   -1.45776035, -1.28407912, -1.10976296, -0.93320981, -0.78664303,
                   -0.64032399, -0.49641576, -0.35973648, -0.2372181 , -0.11551886,
                   0.        ,  0.11017435,  0.21617602,  0.31761684,  0.41321325,
                   0.50795846,  0.59720908,  0.67757566,  0.76429283,  0.84583896,
                   0.91488623,  0.99029698,  1.06669404,  1.1932163 ,  1.31802234,
                   1.47029037,  1.62246692,  1.78785387,  1.95960917,  2.15922111])


        ne = normal_estimation.Normals2d(9, len(self.z))
        self.n = empty([len(self.z),2])
        for i in range(len(self.z)):
            self.n[i] = ne.computeNormal(self.z,self.x,i)

        for i in range(len(self.z)):
            if not math.isnan(self.z[i]): break

        v1 = self.add(self.z[i],self.x[i])

        v1.addPlaneParam(self.n[i,0],self.n[i,1])

        if(i-1<0 or math.isnan(self.z[i-1])):
            # add perpendicular plane
            v1.addPlaneParam(-self.n[i,1],self.n[i,0])
        else:
            # add normal of edge
            dx = self.z[i-1] - self.z[i]
            dy = self.x[i-1] - self.x[i]
            l = 1./math.sqrt(dx**2+dy**2)
            v1.addPlaneParam(-dy*l,dx*l)

        if(i+1>=len(self.z) or math.isnan(self.z[i+1])):
            # add perpendicular plane
            v1.addPlaneParam(-self.n[i,1],self.n[i,0])
        else:
            # add normal of edge
            dx = self.z[i+1] - self.z[i]
            dy = self.x[i+1] - self.x[i]
            l = 1./math.sqrt(dx**2+dy**2)
            v1.addPlaneParam(-dy*l,dx*l)


        for j in range(i+1, len(self.z)):
            v2 = self.add(self.z[j],self.x[j])

            v2.addPlaneParam(self.n[j,0],self.n[j,1])

            if(j-1<0 or math.isnan(self.z[j-1])):
                # add perpendicular plane
                v2.addPlaneParam(-self.n[j,1],self.n[j,0])
            else:
                # add normal of edge
                dx = self.z[j-1] - self.z[j]
                dy = self.x[j-1] - self.x[j]
                l = 1./math.sqrt(dx**2+dy**2)
                v2.addPlaneParam(-dy*l,dx*l)

            if(j+1>=len(self.z) or math.isnan(self.z[j+1])):
                # add perpendicular plane
                v2.addPlaneParam(-self.n[j,1],self.n[j,0])
            else:
                # add normal of edge
                dx = self.z[j+1] - self.z[j]
                dy = self.x[j+1] - self.x[j]
                l = 1./math.sqrt(dx**2+dy**2)
                v2.addPlaneParam(-dy*l,dx*l)


            e = self.connect(v1,v2)
            v1 = v2

    def test_show(self):
        fig = plt.figure(figsize=(1024.0/80, 768.0/80), dpi=80)
        ax = fig.add_subplot(111)
        ax.axis('equal')
        self.draw(ax, 've')

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

            data.vnew = Vertex(float(v[0]),float(v[1]),Q)
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
            disp(cost)
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


m = Mesh()
m.test_load()
m.test_show()
s = Simplifier()
s.init(m)
s.simplify()
m.test_show()
plt.show()
