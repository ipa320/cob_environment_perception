#!/usr/bin/python

import os, sys


assert(len(sys.argv)==3)

f_fts = open(sys.argv[1],'r')
f_tfs = open(sys.argv[2],'r')

DL='\t'

tfs=[]
tfs_file=""
for l in f_tfs:
	s = l.split(DL)
	if len(s)>0: s[len(s)-1] = s[len(s)-1][0:len(s[len(s)-1])-1]
	if len(s)>0 and s[0]=="tf": tfs.append(s[1:])
	if len(s)>0 and s[0]=="file": tfs_file = s[1]

fts_file=""
for l in f_fts:
	s = l.split(DL)
	if len(s)>0: s[len(s)-1] = s[len(s)-1][0:len(s[len(s)-1])-1]

	if len(s)>0 and s[0]=="file": fts_file = s[1]
	if len(s)>0 and s[0]=="header":
		ts = float(s[1])
		best=None
		mi=0.05 #minimum
		for t in tfs:
			if abs(float(t[0])-ts)<mi:
				mi = abs(float(t[0])-ts)
				best = t
		if best==None:
			print "skipping frame as no valid tf was found"
			continue
		print mi


print tfs_file
assert(tfs_file==fts_file)
