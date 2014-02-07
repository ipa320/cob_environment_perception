#!/usr/bin/python

from numpy import *
from matplotlib import *

def tri_dist(t1,t2,n,p):
    #p[0:2]
    array([[n[0]],[n[1]]])
    pp = p[0:2] -  (p.T.dot(n) * n)[0:2]
    disp(pp)

a = array([[3],[1],[1]])
b = array([[3],[4],[1]])
m = array([[1],[0],[-3]])

x = array([[1],[3],[1]])
y = array([[4],[-1],[1]])
tri_dist(a,b,m,x)
