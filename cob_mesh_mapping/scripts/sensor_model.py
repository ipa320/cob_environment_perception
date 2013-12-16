#!/usr/bin/python
from numpy import *
import matplotlib.pyplot as plt

class SensorModel:
    def __init__(self):
        self.P = array([452.705, -611.068, 255.254, -7.295, 7.346])
        self.Q = array([-326.149, 588.446, -548.754, 340.178, -47.175])
        self.a = -1./348.
        self.b = 1090./348.
        self.U = 1.051**2 * (1./582.64)**2
        self.V = 0.801**2 * (1./586.97)**2
        self.D = 1.266**2

    def fd_rational(self, d):
        num = 0.0
        denom = 0.0
        #for i in range(len(self.P)):
        #    dd = d**i
        #    num = num + self.P[i] * dd
        #    denom = denom + self.Q[i] * dd

        return 1./(-0.00287*d + 3.132)
        #return num / denom

    def fd_approx(self, d):
        # b = 7.5cm (base line)
        # f = 580px (focal length IR)
        # doff = 1090 (disparity offset)
        # z = b * f / (doff - d)
        return 1./(self.a*d + self.b)

    def fz_approx(self, z):
        # f(z) -> d
        return (1. - self.b*z)/(self.a*z)

    def fdd_approx(self, d):
        # f(d)' -> z
        res = -self.a/(self.a**2*d**2 + 2.*self.a*self.b*d + self.b**2)
        return res

    def covariance(self, mu):
        # fdd_approx could be cached for d=[400..1050]
        fdd = self.fdd_approx(round(self.fz_approx(mu[2])))
        dd  = self.D * fdd**2
        zz  = mu[2]**2
        dxx = dd * mu[0]**2
        dxy = dd * mu[0]*mu[1]
        dyy = dd * mu[1]**2
        dx  = dd * mu[0]
        dy  = dd * mu[1]

        sigma = array([[self.U*zz + dxx, dxy, dx],
                       [dxy, self.V*zz + dyy, dy],
                       [dx, dy, dd]])
        return sigma

    def propability(self, p, sigma):
        """propability of point p given unbiased normal distribution sigma"""
        ex = p.T.dot(linalg.inv(sigma)).dot(p)
        ex = math.exp(-0.5 * ex)
        disp(ex)
        coef = 1.0/math.sqrt((2.*pi)**3 * linalg.det(sigma))
        disp(coef)
        return coef*ex


s = SensorModel()
fig = plt.figure()
ax = fig.add_subplot(111)
d = range(400,1000)
z1 = empty(len(d))
z2 = empty(len(d))
for i in range(len(d)):
    z1[i] = s.fd_approx(d[i])
    z2[i] = s.fdd_approx(d[i])

#ax.plot(d,z1,'-')
#ax.plot(d,z2,'-')
#ax.grid()
#plt.show()

off = array([[0],[0],[0.01]])
m1 = array([[0.3474],[0.2842],[1.1917]])
c1 = s.covariance(m1)
disp(c1 * 10000)
disp(s.propability(off, c1))
m2 = array([[-0.8402],[0.3473],[2.0383]])
c2 = s.covariance(m2)
disp(c2 * 1000)
disp(s.propability(off, c2))

