#!/usr/bin/python

from numpy import *
import matplotlib.pyplot as plt
import normal_estimation
#reload(normal_estimation)


def computeNormal(x1,y1,x2,y2):
    dx = x2 - x1
    dy = y2 - y1
    l = 1./math.sqrt(dx**2+dy**2)
    return (-dy*l, dx*l)

class Vertex:
    def __init__(self, x, y, q = zeros([3,3]), w = 0.0):
        """ """
        self.x = x
        self.y = y
        self.Q = w * q
        self.w = w
        self.e1 = None
        self.e2 = None

    def addPlaneParam(self, nx, ny, w = 1.0):
        """ """
        #self.nx = nx
        #self.ny = ny
        d = -(nx*self.x + ny*self.y)
        q = array([[nx],[ny],[d]])
        Q = q.dot(q.T)
        #a = math.degrees(math.atan2(ny, nx))
        #if a < 0: print a+180
        #else: print a
        self.w = self.w + w
        self.Q = self.Q + w*Q

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
        """performs edge collapse operation"""
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
        """performs edge split operation"""
        pass

    def move(self, v):
        """performs vertex move operation"""
        pass

    def load(self,x,y,nx,ny):
        """creates mesh from measurements and normals"""
        for i in range(len(x)):
            if not math.isnan(x[i]): break

        pen=100.0
        v1 = self.add(x[i],y[i])
        v1.addPlaneParam(nx[i],ny[i],2.0)

        if(i-1<0 or math.isnan(x[i-1])):
            # add perpendicular plane
            v1.addPlaneParam(-ny[i],nx[i],pen)
            print v1, v1.w
        else:
            # add normal of edge
            nix,niy = computeNormal(x[i-1], x[i], y[i-1], y[i])
            v1.addPlaneParam(nix,niy)

        if(i+1>=len(x) or math.isnan(x[i+1])):
            # add perpendicular plane
            v1.addPlaneParam(-ny[i],nx[i],pen)
            print v1, v1.w
        else:
            # add normal of edge
            nix,niy = computeNormal(x[i], x[i+1], y[i], y[i+1])
            v1.addPlaneParam(nix,niy)


        for j in range(i+1, len(x)):
            v2 = self.add(x[j],y[j])
            v2.addPlaneParam(nx[j],ny[j],2.0)

            if(j-1<0 or math.isnan(x[j-1])):
                # add perpendicular plane
                v2.addPlaneParam(-ny[j],nx[j],pen)
                print v2, v2.w
            else:
                # add normal of edge
                nix,niy = computeNormal(x[j-1], x[j], y[j-1], y[j])
                v2.addPlaneParam(nix,niy)

            if(j+1>=len(x) or math.isnan(x[j+1])):
                # add perpendicular plane
                v2.addPlaneParam(-ny[j],nx[j],pen)
                print v2, v2.w
            else:
                # add normal of edge
                nix,niy = computeNormal(x[j], x[j+1], y[j], y[j+1])
                v2.addPlaneParam(nix,niy)

            e = self.connect(v1,v2)
            v1 = v2


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

        self.load(self.z,self.x,self.n[:,0],self.n[:,1])

    def test_show(self):
        fig = plt.figure(figsize=(1024.0/80, 768.0/80), dpi=80)
        ax = fig.add_subplot(111)
        ax.axis('equal')
        self.draw(ax, 've')




#m = Mesh()
#m.test_load()
#m.test_show()
#plt.show()
