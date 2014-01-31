#!/usr/bin/python

from numpy import *

def make_affine(vectors):
    return hstack((vectors,ones([len(vectors),1])))

def transform(tf, vectors):
    return vstack(([tf.dot(v) for v in vectors]))
