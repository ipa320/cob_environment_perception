#!/usr/bin/python
from numpy import *
import matplotlib.pyplot as plt

class SensorModel:
    def __init__(self):
        self.P = array([452.705, -611.068, 255.254, -7.295, 7.346])
        self.Q = array([-326.149, 588.446, -548.754, 340.178, -47.175])
        self.a = -1./(8. * 0.075 * 580.) # -1/(8*base*f_0)
        self.b = 1090./(8. * 0.075 * 580.) # d_off / (8*base*f_0)
        self.U = 1.051**2 * (1./582.64)**2
        self.V = 0.801**2 * (1./586.97)**2
        self.D = 1.266**2

    def fd_rational(self, d):
        num = 0.0
        denom = 0.0
        for i in range(len(self.P)):
            dd = d**i
            num = num + self.P[i] * dd
            denom = denom + self.Q[i] * dd

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
        #res = -self.a/(self.a**2*d**2 + 2.*self.a*self.b*d + self.b**2)
        res = -self.a/(self.a * d + self.b)**2
        return res

    def covariance(self, mu):
        # fdd_approx could be cached for d=[400..1050]
        x_z = float(mu[0] / mu[2])
        y_z = float(mu[1] / mu[2])
        d = round(self.fz_approx(mu[2]))
        fdd = float(self.fdd_approx(d))
        dd  = float(self.D * fdd**2)
        zz  = float(mu[2]**2)
        dxx = dd * x_z**2
        dxy = dd * x_z*y_z
        dyy = dd * y_z**2
        dx  = dd * x_z
        dy  = dd * y_z

        sigma = array([[self.U*zz + dxx, dxy, dx],
                       [dxy, self.V*zz + dyy, dy],
                       [dx, dy, dd]])
        return sigma

    def log_probability(self, p, sigma):
        """probability of point p given unbiased normal distribution sigma"""
        ex = -0.5 * p.T.dot(linalg.inv(sigma)).dot(p)
        coef = 1./math.sqrt(linalg.det(2.*pi*sigma))
        interval = 0.001**3
        return coef * math.exp(ex) * interval


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
c11 = array([[0.0693, 0.0189, 0.0793],
             [0.0189, 0.0419, 0.0649],
             [0.0793, 0.0649, 0.2722]]) * 10**(-4)
disp(c1 * 10**(4))
disp(s.log_probability(off, c1))
disp(c11 * 10**(4))
disp(s.log_probability(off, c11))
m2 = array([[-0.8402],[0.3473],[2.0383]])
c2 = s.covariance(m2)
c22 = array([[0.0531, -0.0163, -0.0959],
            [-0.0163, 0.0145, 0.0397],
            [-0.0959, 0.0397, 0.2327]]) * 10**(-3)

disp(c2 * 1000)
disp(s.log_probability(off, c2))
disp(c22 * 1000)
disp(s.log_probability(off,c22))
