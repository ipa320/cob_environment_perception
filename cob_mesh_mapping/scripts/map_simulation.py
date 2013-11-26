from numpy import *
import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

def make_affine(vectors):
    return hstack((vectors,ones([len(vectors),1])))

def transform(tf, vectors):
    return vstack(([tf.dot(v) for v in vectors]))

### BEGIN CLASS -- Map ###
class World:

    def __init__(self, coords):
        self.coords = coords

### END CLASS -- Map ###

### BEGIN CLASS -- Sensor ###
class Sensor:

    def __init__(self, position, orientation):
        self.pos = array(position)
        self.dir = array(orientation) / linalg.norm(orientation)

        # Field of view properties;
        fov = self.fov = 49.0 / 180.0 * pi # fov angle
        tan_fov_2 = tan(fov * 0.5)
        f = self.f = 10.0 # far plane
        n = self.n = 1. # near plane
        l = self.l = n * tan_fov_2 # right
        r = self.r = -l # left
        self.frustum = array([[n,l,1], [f, f*tan_fov_2,1],
                              [f,-f*tan_fov_2,1], [n,r,1]])

        # perspectiv projection matrix:
        depth = f - n # f > n
        width = l - r # l > r
        # scale rectangle to unit cube
        us = array([[2./depth,0,0],
                    [0,2./width,0],
                    [0,0,1.]])
        # translate rectangle center to origin
        ut = array([[1,0,-(n+depth/2.)],
                    [0,1,-(r+width/2.)],
                    [0,0,1.]])
        # translate back to position
        txn1 = array([[1.,0,n],
                      [0,1.,0],
                      [0,0,1.]])
        # scale to original size
        sxfn = array([[f/n,0,0],
                      [0,1.,0],
                      [0,0,1.]])
        # project frustum to rectangle
        pxn = array([ [1.,0,0],
                      [0,1.,0],
                      [1./n,0,1.]])
        # translate frustum to origin
        txn2 = array([[1.,0,-n],
                      [0,1.,0],
                      [0,0,1.]])
        # combine everything
        self.pp = us.dot(ut).dot(txn1).dot(sxfn).dot(pxn).dot(txn2)
        #self.pp = txn1.dot(sxfn).dot(pxn).dot(txn2)

        # rotation and translation matrix:
        phi = math.atan2(orientation[1], orientation[0])
        sign = math.copysign(1,phi)
        phi = fabs(phi)
        self.tf_to_world = array([[cos(phi), -sign * sin(phi), position[0]],
                                  [sign * sin(phi), cos(phi), position[1]],
                                  [0, 0, 1.]])
        self.tf_to_cam = linalg.inv(self.tf_to_world)

    def measure(self, world):
        w = make_affine(world.coords)
        self.world = transform(self.tf_to_cam,w)
        w = transform(self.pp.dot(self.tf_to_cam), w)
        w = vstack(v / v[-1] for v in w)

        y = array(range(-100,100,5)) * 0.01
        x = ones(len(y))*10
        p = empty([len(y)-1, 2]) # m_inv, t
        for i in range(len(w)-1):
            d = w[i+1] - w[i]
            p[i] = [d[0] / d[1], d[0] * w[i][1] / (d[1] * w[i][0])]

        for i in range(len(y)):
            for pi in p:
                tmp = pi[0] * (y[i] - pi[1])
                if tmp > -1. and tmp < 1.:
                    x[i] = min(tmp, x[i])

        self.axis = plt.figure().add_subplot(111)
        self.axis.set_xlim(-1., 1.)
        self.axis.set_ylim(-1, 1.)
        self.axis.plot(w[:,0],w[:,1])
        self.axis.plot(x,y,'x')

        back = linalg.inv(self.pp)
        vst = vstack(zip(x,y,ones(len(x))))
        self.measurement = vstack(v/v[-1] for v in transform(back,vst))

    def draw(self, axis):
        #x, y = zip(self.pos)
        #dx, dy = zip(self.dir)
        vfrustum = transform(self.tf_to_world, self.frustum)[:,0:2]
        poly = mpatches.Polygon(vfrustum, alpha=0.2, fc=(0,.75,0))
        axis.add_patch(poly)
        axis.plot(self.pos[0],self.pos[1], 'o',
                  markeredgecolor=(0,.75,0), markerfacecolor=(0,.75,0),
                  markersize=10)
        #axis.quiver(x, y, dx, dy, angles='xy', scale_units='xy', scale=1,
        #            facecolor='none', edgecolor='red', linewidth=1)

    def showMeasurement(self):
        self.axis = plt.figure().add_subplot(111)
        self.axis.plot(self.measurement[:,0],self.measurement[:,1],'x')
        #self.axis.plot(self.world[:,0],self.world[:,1])

    def showMeasurementInMap(self, axis):
        transformed = transform(self.tf_to_world,self.measurement)
        axis.plot(transformed[:,0],transformed[:,1], 'x')

### END CLASS -- Map ###


m = World(array(
    [[-15., 13.],
     [-15., -5.],
     [-10.,-10.],
     [ -1.,-10.],
     [ -1., -5.],
     [ -5., -5.],
     [ -5., -4.],
     [  1., -4.],
     [  1.,-10.],
     [ 12.,-10.],
     [ 18., -2.],
     [ 12.,  1.],
     [ 18.,  4.],
     [ 12.,  7.],
     [ 18., 10.],
     [ 12., 13.]]))

sensors = [Sensor([-8., -7.5],[1.,0.1]),
           Sensor([-7.5, -3.],[ 1.,-0.8]),
           Sensor([-4.5, -0.5],[0.,-1.]),
           Sensor([ 3., -1.],[-1.,-1.])]


fig = plt.figure(figsize=(1024.0/80, 768.0/80), dpi=80)
fig
ax = fig.add_subplot(111)
#ax.plot(m.coords[:,0], m.coords[:,1], 'x-', lw=2, color='black', ms=10)

for s in sensors:
    s.draw(ax)
    s.measure(m)
    #s.showMeasurement()
    #s.showMeasurementInMap(ax)


ax.plot(m.coords[:,0], m.coords[:,1], 'ko-')
ax.axis('equal')
ax.set_xlim(-20, 20)
ax.set_ylim(-15, 15)
ax.grid()

plt.show()
