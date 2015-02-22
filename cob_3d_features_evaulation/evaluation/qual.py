#!/usr/bin/python

from num_frames import num_frames
import csv, tempfile
import os, sys, math

def qual_matrix(files, ft, ind=6):
	MIN_DIST=0.25
	IND=ind
	FACTOR_IND=ind
	radius="0.7"

	R={}
	frames_whole=0
	for base_name in files:
		if not os.path.isfile(base_name):
			print "WARNING: missing file "+base_name
			continue
			
		frames, kp = num_frames(base_name)
		frames_whole += frames

		fn = base_name+"-feature-"+radius+"_"+ft+".csv"
		if not os.path.isfile(fn):
			fn = base_name[:len(base_name)-4]+"-feature-"+radius+"_"+ft+".csv"
		if not os.path.isfile(fn):
			print "WARNING: missing file "+fn
			continue
			
		with open(fn, 'rb') as csvfile:
			data = csv.reader(csvfile, delimiter='\t', quotechar='|')
			M = []
			for row in data:
				if float(row[0])>=MIN_DIST: M.append(row)
				
			print ft, "\t", M[5][3], "\t", frames, kp, "    ",fn
			
			if float(M[len(M)-1][FACTOR_IND])==0:
				M.remove(M[len(M)-1])
				
			for row in M:
				if ind<7:
					v = frames * float(row[IND])/float(M[len(M)-1][FACTOR_IND])
				else:
					v = frames * float(row[IND])
				i = float(row[0])
				if i in R:
					R[i] += v
				else:
					R[i] = v
				
	for	i in R: R[i]/=frames_whole
	#print ft
	#for i in sorted(R):
	#	print i, "\t",R[i]
		
	return R

def time_matrix(files, ft="timing"):
	MIN_DIST=0.25
	FACTOR_IND=3
	radius="0.7"

	R=0
	frames_whole=0
	for base_name in files:
		if not os.path.isfile(base_name):
			print "WARNING: missing file "+base_name
			continue
			
		frames, kp = num_frames(base_name)
		frames_whole += frames

		fn = base_name+"-feature-"+radius+"_"+ft+".csv"
		if not os.path.isfile(fn):
			fn = base_name[:len(base_name)-4]+"-feature-"+radius+"_"+ft+".csv"
		if not os.path.isfile(fn):
			print "WARNING: missing file "+fn
			continue
			
		with open(fn, 'rb') as csvfile:
			data = csv.reader(csvfile, delimiter='\t', quotechar='|')
			M = []
			for row in data:
				M.append(row)
			
			for row in M:
				R += frames * float(row[FACTOR_IND])
				break
				
	R/=frames_whole
		
	return R
