#!/usr/bin/python

from numpy import *
import matplotlib.patches as mpatches
from tf_utils import *

def getPoseVectorField(objects):
    vf = array([[s.pos[0],s.pos[1],s.ori[0],s.ori[1]] for s in objects])
    return vf

def drawPoses(objects,axis):
    vf = array([[s.pos[0],s.pos[1],s.ori[0],s.ori[1]] for s in objects])
    axis.quiver(vf[:,0],vf[:,1],vf[:,2],vf[:,3],facecolor='r',
                width=0.002,headlength=8,headwidth=6,headaxislength=8)


class Camera2d:
    """ res: number of pixels (multiple of 2)"""
    def __init__(self, fov, far, near, res = 48.):
        tan_fov_2 = tan(fov*0.5)
        self.res = res
        self.fov = fov
        self.f = f = far
        self.n = n = near
        self.l = l = n * tan_fov_2
        self.r = r = -l
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

        self.tf_to_unit_cube = us.dot(ut).dot(txn1).dot(sxfn).dot(pxn).dot(txn2)
        self.tf_to_frustum = linalg.inv(self.tf_to_unit_cube)

    def setPose(self, position, orientation):
        self.pos = array(position)
        self.ori = array(orientation) / linalg.norm(orientation)

        # rotation and translation matrix:
        phi = math.atan2(orientation[1], orientation[0])
        sign = math.copysign(1,phi)
        phi = fabs(phi)
        self.tf_to_world = array([[cos(phi), -sign * sin(phi), position[0]],
                                  [sign * sin(phi), cos(phi), position[1]],
                                  [0, 0, 1.]])
        self.tf_to_cam = linalg.inv(self.tf_to_world)

    def drawFrustum(self, axis):
        vfrustum = transform(self.tf_to_world, self.frustum)[:,0:2]
        poly = mpatches.Polygon(vfrustum, alpha=.2, fc=(0,.75,0))
        axis.add_patch(poly)

    def drawPose(self,axis):
        axis.plot(self.pos[0],self.pos[1], 'o',
                  markeredgecolor=(0,0,0), markerfacecolor=(0,0,0),
                  markersize=2)
