#!/usr/bin/python

from numpy import *

class Normals2d:

    def __init__(self, window_size):
        """w: window size"""
        self.mask = range(-window_size/2,window_size/2)

    def computeNormal(z,i):
        """ """
        for o in self.mask:
            idx = i+o
            if idx < 0 or idx >= len(z): continue
            if math.isnan(z[idx]): continue


    def test(self):
        """function for self validation"""
        z = array([ nan,         nan,         nan,  8.07947014,  7.78973796,
                    7.52006616,  7.26844107,  7.03310978,  6.81253928,  6.60538303,
                    6.41045344,  6.22669908,  6.05318574,  5.88908048,  5.73363833,
                    5.58619094,  5.44613699,  5.312934  ,  5.18609126,  5.06516388,
                    4.94974747,  4.83947371,  4.73400637,  4.63303792,  4.5362865 ,
                    4.44349332,  4.35442036,  4.26884828,  4.18657467,  4.10741241,
                    4.03118829,  3.95774172,  3.89287959,  4.01891987,  4.1533949 ,
                    4.29718067,  4.45127883,  4.61684008,  4.7951929 ,  4.98787931])
    n = empty(len(z))
    for i in range(len(z)):
        n[i] = computeNormal(z, i)

ne = Normals2d(5)
ne.test()
