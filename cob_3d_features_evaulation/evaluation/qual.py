#!/usr/bin/python

from num_frames import num_frames
import csv, tempfile
import os, sys, math

def qual_matrix(files, ft):
	MIN_DIST=0.15
	FACTOR_IND=4
	radius="0.7"

	R={}
	frames_whole=0
	for base_name in files:
		frames = num_frames(base_name)
		frames_whole += frames

		fn = base_name+"-feature-"+radius+"_"+ft+".csv"
		if not os.path.isfile(fn):
			print "WARNING: missing file "+fn
			continue
			
		with open(fn, 'rb') as csvfile:
			data = csv.reader(csvfile, delimiter='\t', quotechar='|')
			M = []
			for row in data:
				if float(row[0])>=MIN_DIST: M.append(row)
			
			for row in M:
				v = frames * float(row[3])/float(M[len(M)-1][FACTOR_IND])
				i = float(row[0])
				if i in R:
					R[i] += v
				else:
					R[i] = v
				
	for	i in R: R[i]/=frames_whole
	print ft
	for i in sorted(R):
		print i, "\t",R[i]
		
	return R
