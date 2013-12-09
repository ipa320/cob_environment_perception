#!/usr/bin/python

from numpy import *


def spherical2d(x,y):
    """return azimuth:-pi..pi"""
    return math.atan2(y,x)

def spherical3d(p):
    """return (azimuth:-pi..pi, polar:0..pi)"""
    x,y,z=p
    return (math.atan2(y,x), arccos(z / linalg.norm(p)))


class Normals2d:

    def __init__(self, window_size):
        """w: window size"""
        r = window_size/2
        self.mask = range(-r,r+1)
        del self.mask[len(self.mask)/2]
        self.hist_size = 11.
        self.amin = 0. #-0.5*pi
        self.amax = pi #0.5*pi
        self.to_bin = self.hist_size/(self.amax - self.amin)

    def bin(self,angle):
        return int((angle - self.amin) * self.to_bin)

    def computeNormal(self,x,z,i):
        """ """
        #angle = []
        for o in self.mask:
            idx = i+o
            if idx < 0 or idx >= len(z): continue
            if math.isnan(z[idx]): continue
            px = x[i] - x[idx]
            py = z[i] - z[idx]
            s = math.copysign(1,px) # flip if x < 0
            #angle.append(spherical2d(s*px,s*py))
            angle = spherical2d(s*px,s*py)+pi/2.
            if math.isnan(angle): continue
            hi = self.bin(angle)
            self.hist_avg[i,(hi+1)%self.hist_size] += angle
            self.hist_avg[i, hi   %self.hist_size] += 2.*angle
            self.hist_avg[i,(hi-1)%self.hist_size] += angle
            self.hist_inc[i,(hi+1)%self.hist_size] += 1
            self.hist_inc[i, hi   %self.hist_size] += 2
            self.hist_inc[i,(hi-1)%self.hist_size] += 1

        #self.angles[i,0:len(angle)] = angle
        maxi = argmax(self.hist_inc[i,:])
        return self.hist_avg[i,maxi] / float(self.hist_inc[i,maxi])


    def test(self):
        """function for self validation"""
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

        weight = array([[0   , .125, 0   ],
                        [.125, .5  , .125],
                        [0   , .125, 0   ]])

        self.normals = empty(len(self.z))
        self.angles = empty([len(self.z), len(self.mask)])
        self.angles[:] = nan
        self.hist_avg = zeros([len(self.z), self.hist_size])
        self.hist_inc = zeros([len(self.z), self.hist_size])
        for i in range(len(self.z)):
            self.normals[i] = self.computeNormal(self.x,self.z,i)


ne = Normals2d(9)
ne.test()
