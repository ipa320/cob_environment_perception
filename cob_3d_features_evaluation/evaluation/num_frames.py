#!/usr/bin/python

import os, sys, math
from cgkit.cgtypes import vec3, vec4, quat
from scipy.stats.mstats import mquantiles

#assert(len(sys.argv)==2)

#@profile
def num_frames(fn):
	f_fts = open(fn,'r')

	DL='\t'

	fts_file=""
	T=None
	keypoints=0
	KPstart=0
	frames=0
	took={}
	for l in f_fts:
		s = l.split(DL)
		if len(s)>0: s[len(s)-1] = s[len(s)-1][0:len(s[len(s)-1])-1]

		if len(s)>0 and s[0]=="file": fts_file = s[1]
		if len(s)>0 and s[0]=="header":
			T=None
			ts = float(s[1])
			best=None
			mi=0.075 #minimum
			KPtype=""
			frames+=1
		if len(s)>0 and s[0]=="took":
			if not s[1] in took: took[s[1]] = []
			took[s[1]].append(reduce(lambda x, y: float(x) + float(y), s[2:]))

		if len(s)>4 and s[0]=="keypoint":
			keypoints+=1

	#print "frames ",frames
	return [frames, keypoints]

#run(sys.argv[1])
