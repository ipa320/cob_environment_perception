#!/usr/bin/python

from numpy import *
import matplotlib.pyplot as plt

def cart_coords(sph):
    """azimuth: -pi..pi, polar: 0..pi"""
    return (sin(sph[1])*cos(sph[0]),
            sin(sph[1])*sin(sph[0]),
            cos(sph[1]))

def sphere_coords(p):
    """return (azimuth:-pi..pi, polar:0..pi)"""
    inv_norm = 1.
    x,y,z = p/linalg.norm(p)
    return (math.atan2(y,x), arccos(z))

def vote(p, phi):
    """return (x,y,z)"""
    x,y,z = p
    denom = 1./sqrt(x**2+y**2)
    sin_phi = sin(phi)
    cos_phi = cos(phi)
    rx = ( x*z*cos_phi + y*sin_phi) * denom
    ry = (-y*z*cos_phi + x*sin_phi) * denom
    rz = sqrt(1.-z**2)*cos_phi

    #az = math.atan2(y,x)
    #ay = arccos(z)
    #Rz = array([[cos(az),-sin(az),0],[sin(az),cos(az),0],[0,0,1]])
    #Ry = array([[cos(ay),0,sin(ay)],[0,1,0],[-sin(ay),0,cos(ay)]])
    #res = linalg.inv(Ry.dot(Rz)).dot(array([[cos_phi],[sin_phi],[0]]))
    #return (res[0],res[1],res[2])
    return (rx,ry,rz)

n = 500
circle = array(range(-n/2,n/2)) / float(n) * 2. * pi
#circle = array(range(n)) / float(n) * pi
#p = array([cart_coords([pi/2.,.1*pi]),
#           cart_coords([pi/2.,.2*pi]),
#           cart_coords([pi/2.,.4*pi]),
#           cart_coords([pi/2.,.6*pi]),
#           cart_coords([pi/2.,.8*pi]),
#           cart_coords([pi/2.,.9*pi])])
p = array([cart_coords([ 1.8*pi/2.,.4*pi]),
           cart_coords([ 1.*pi/2.,.4*pi]),
           cart_coords([ 0.*pi/2.,.4*pi]),
           cart_coords([-1.*pi/2.,.4*pi]),
           cart_coords([-1.8*pi/2.,.4*pi]),
           cart_coords([pi/2.,.1*pi]),
           cart_coords([pi/2.,.2*pi]),
           cart_coords([pi/2.,.4*pi]),
           cart_coords([pi/2.,.6*pi]),
           cart_coords([pi/2.,.8*pi]),
           cart_coords([pi/2.,.9*pi])])
p = vstack([ x / linalg.norm(x) for x in p ])
spheres = empty([len(p), len(circle), 2])
sph_cart = empty([len(p), len(circle), 3])

for i in range(len(circle)):
    for j in range(len(p)):
        sph_cart[j,i,:] = x,y,z = vote(p[j],circle[i])
        #spheres[j,i,:] = (y / x, z)
        spheres[j,i,:] = sphere_coords(sph_cart[j,i,:])

fig = plt.figure(figsize=(1536.0/80, 768.0/80), dpi=80)
ax = fig.add_subplot(111)
for i in range(len(p)):
    ax.plot(spheres[i,:,0],spheres[i,:,1],'x')
    #ax.plot(sph_cart[i,:,0],sph_cart[i,:,1], 'x')
    #ax.plot(p[i,0],p[i,1],'ok')
    inter_x = cross(p[i],p[(i+1)%len(p)])
    inter_a = sphere_coords([inter_x[0], inter_x[1], -inter_x[2]])
    ax.plot(inter_a[0],inter_a[1],'ok')


ax.axis('equal')
ax.set_xlim(-pi, pi)
ax.set_ylim(0, pi)
ax.grid()
plt.show()
