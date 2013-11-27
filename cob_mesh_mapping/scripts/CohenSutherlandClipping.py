#!/usr/bin/python

from numpy import *

class Clipper:

    def __init__( self, lim = array([[-1,1],[-1,1]]) ):
        self.lim = lim

    def compute_code(self, p):
        c = 0
        for i in range(len(self.lim)):
            if p[i] < self.lim[i][0]:
                c |= 0b1 << 2*i
            elif p[i] > self.lim[i][1]:
                c |= 0b1 << 2*i+1
        return c

    def clip(self, p0, p1):
        c0 = self.compute_code(p0)
        return self.clip_recursiv(p0, p1, c0)

    def clip_recursiv(self, p0, p1, c0):
        c1 = self.compute_code(p1)

        # all zero => trivial accept:
        if not (c0 | c1): return (True, p0, p1)
        # both points share on side => trivial reject:
        if (c0 & c1): return (False, p0, p1)

        # pick one point that lies outside:
        if c0:
            cnew = c0
            cold = c1
            pold = p1
        else:
            cnew = c1
            cold = c0
            pold = p0

        for i in range(len(self.lim)):
            if cnew & (0b1 << 2*i):
                d = (self.lim[i][0] - p0[i]) / (p1[i] - p0[i])
                pnew = (p1 - p0) * d + p0
                pnew[i] = self.lim[i][0]
                break
            elif cnew & (0b1 << 2*i+1):
                d = (self.lim[i][1] - p0[i]) / (p1[i] - p0[i])
                pnew = (p1 - p0) * d + p0
                pnew[i] = self.lim[i][1]
                break

        return self.clip_recursiv(pold, pnew, cold)

    def test(self):
        print bin(self.compute_code([ 0, 0])), " 00 00 "
        print bin(self.compute_code([-2, 0])), " 00 01 "
        print bin(self.compute_code([ 2, 0])), " 00 10 "
        print bin(self.compute_code([ 0,-2])), " 01 00 "
        print bin(self.compute_code([ 0, 2])), " 10 00 "
        print bin(self.compute_code([-2,-2])), " 01 01 "
        print bin(self.compute_code([ 2, 2])), " 10 10 "
        print bin(self.compute_code([-2, 2])), " 10 01 "
        print bin(self.compute_code([ 2,-2])), " 01 10 "
        print self.clip(array([0,-2]), array([0,2])), "(0,-2) (0,2)"
        print self.clip(array([-2,0]), array([2,0])), "(-2,0) (2,0)"
        print self.clip(array([-2,3]), array([2,0])), "(-2,3) (2,0)"
        print self.clip(array([-3,-1]), array([3,1])), "(-3,-1) (3,1)"
        print self.clip(array([-2,3]), array([0,0])), "(-2,3) (2,0)"
        print self.clip(array([0,0]), array([3,1])), "(-3,-1) (3,1)"
        print self.clip(array([1.1,-0.3]), array([0.9,0.8])), "(-3,-1) (3,1)"

