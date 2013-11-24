from numpy import *
import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

### BEGIN CLASS -- Map ###
class Map:

    def __init__(self, coords):
        self.coords = coords

### END CLASS -- Map ###

### BEGIN CLASS -- Sensor ###
class Sensor:

    def __init__(self, position, orientation):
        self.pos = array(position)
        self.dir = array(orientation) / linalg.norm(orientation)
        fov = 49.0 / 180.0 * pi; # fov angle
        f = 5.0; # far plane
        n = 0.5; # near plane
        r = n * tan(fov * 0.5); # right
        l = -r; # left
        mortho = array([[2.0/(n-f),0,0],
                        [0,2.0/(r-l),0],
                        [0,0,1]])
        txn1 = array([[1,0,n],
                      [0,1,0],
                      [0,0,1]])
        sxfn = array([[f/n,0,0],
                      [0,1,0],
                      [0,0,1]])
        pxn = array([[1,0,0],
                     [0,1,0],
                     [1/n,0,1]])
        txn2 = array([[1,0,-n],
                      [0,1,0],
                      [0,0,1]])
        self.pp = mortho * txn1 * sxfn * pxn * txn2 # perspectiv projection

        phi = math.atan2(orientation[1], orientation[0])
        self.tf = array([[cos(phi),-sin(phi), position[0]],
                         [sin(phi), cos(phi), position[1]],
                         [0, 0, 1]])

    def draw(self, axis):
        axis.quiver(self.pos[0], self.pos[1], self.dir[0], self.dir[1],
                    angles='xy', scale_units='xy', scale=1,
                    facecolor='none', edgecolor='red', linewidth=1)

### END CLASS -- Map ###


m = Map(array(
    [[-3,-3],
     [ 1, 1],
     [ 4, 1],
     [ 4, 3],
     [ 6, 3]]))

sensors = [Sensor([0, 2],[ 1,-1]),
           Sensor([5, 5],[-1,-1])]


fig = plt.figure(figsize=(1024.0/80, 768.0/80), dpi=80)
ax = fig.add_subplot(111)
#ax.plot(m.coords[:,0], m.coords[:,1], 'x-', lw=2, color='black', ms=10)

for s in sensors:
    s.draw(ax)

ax.plot(m.coords[:,0], m.coords[:,1], 'ko-')
ax.axis('equal')
ax.set_xlim(-20, 20)
ax.set_ylim(-15, 15)
ax.grid()

plt.show()
