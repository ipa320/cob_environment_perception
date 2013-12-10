from numpy import *
import matplotlib.pyplot as plt
import normal_estimation
reload(normal_estimation)

class Vertex:
    def __init__(self, x, y, q = matrix(empty([3,3])) ):
        """ """
        self.x = x
        self.y = y
        self.Q = q
        self.e1 = None
        self.e2 = None


    def init(self, nx, ny):
        """ """
        self.nx = nx
        self.ny = ny
        d = -(nx*self.x + ny*self.y)
        q = array([[nx],[ny],[d]])
        self.Q = q.dot(q.T)

    def __repr__(self):
        return "(%3.2f %3.2f)" % (self.x, self.y)

class Edge:
    def __init__(self, v1, v2):
        """ """
        self.v1 = v1
        self.v2 = v2
        self.cost = None
        self.vnew = v1
        self.dirty = False
        #disp(self.cost)

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

    def test(self):
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
        n = []
        for i in range(len(self.z)):
            n.append(ne.computeNormal(self.z,self.x,i))

        v1 = self.add(self.z[0],self.x[0])
        for j in range(i+1, len(self.z)):
            v2 = self.add(self.z[j],self.x[j])
            v2.init(n[0],n[1])
            e = self.connect(v1,v2)
            v1 = v2

        fig = plt.figure(figsize=(1024.0/80, 768.0/80), dpi=80)
        ax = fig.add_subplot(111)
        ax.axis('equal')
        self.draw(ax, 've')


class Simplifier:
    def __init__(self):
        self.heap = 0

    def update(self, e):
        Q = e.v1.Q + e.v2.Q
        q = array(vstack([ Q[0:2,:], [0,0,1] ]))
        v = linalg.inv(q).dot(array([[0],[0],[1]]))
        e.cost = float(v.T.dot(Q).dot(v))
        if e.cost > 0.001:
            e.vnew = Vertex(float(v[0]),float(v[1]),Q)
        else:
            e.vnew = Vertex(0.5 * (e.v1.x + e.v2.x),
                            0.5 * (e.v1.y + e.v2.y), Q)


    def test(self):
        pass


m = Mesh()
m.test()
plt.show()
