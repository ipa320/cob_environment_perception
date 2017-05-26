#!/usr/bin/python

import os, sys, math
from cgkit.cgtypes import vec3, vec4, quat
from scipy.stats.mstats import mquantiles

assert(len(sys.argv)==4)

#@profile
def run():
	f_fts = open(sys.argv[1],'r')
	f_tfs = open(sys.argv[2],'r')
	fn_output = sys.argv[3]

	DL='\t'

	tfs=[]
	tfs_file=""
	for l in f_tfs:
		s = l.split(DL)
		if len(s)>0: s[len(s)-1] = s[len(s)-1][0:len(s[len(s)-1])-1]
		if len(s)>0 and s[0]=="tf": tfs.append(s[1:])
		if len(s)>0 and s[0]=="file": tfs_file = s[1]

	fts_file=""
	T=None
	frames=[]
	took={}
	radius=0
	for l in f_fts:
		s = l.split(DL)
		if len(s)>0: s[len(s)-1] = s[len(s)-1][0:len(s[len(s)-1])-1]

		if len(s)>0 and s[0]=="file": fts_file = s[1]
		if len(s)>0 and s[0]=="header":
			T=None
			ts = float(s[1])
			radius = None
			best=None
			mi=0.075 #minimum
			for t in tfs:
				if abs(float(t[0])-ts)<mi:
					mi = abs(float(t[0])-ts)
					best = t
			if best==None:
				#print "skipping frame as no valid tf was found"
				continue

			T = [vec4(float(best[1]), float(best[2]), float(best[3]), 1.), quat(float(v) for v in best[4:])]
			frames.append({})
		elif radius==None:
			radius = float(s[1])

		if len(s)>0 and s[0]=="took":
			name = '_'.join(s[1:4])
			if not name in took: took[name] = []
			if not name in frames[len(frames)-1]: frames[len(frames)-1][name] = []
			took[name].append(float(reduce(lambda x, y: float(x) + float(y), s[4:])))

		if len(s)==7 and s[0]=="eval_keypoint" and T!=None:
			name = '_'.join(s[1:4])
			pt = vec4(float(s[4]), float(s[5]), float(s[6]), 1.)
			TT=T[1].toMat4().setColumn(3, T[0])
			#pt = TT.inverse()*pt
			pt = TT*pt
			frames[len(frames)-1][name].append({"pt":pt})

	print "read input file "

	f_out = file(fn_output+"_timing.csv", "w")
	i=1
	for ft in took:
		aa,bb,cc = mquantiles(took[ft])
		f_out.write( str(i)+"\t" )
		f_out.write( str(min(took[ft]))+"\t")
		f_out.write( str(aa)+"\t" )
		f_out.write( str(bb)+"\t" )
		f_out.write( str(cc)+"\t" )
		f_out.write( str(max(took[ft]))+"\t" )
		f_out.write( str(reduce(lambda x, y: x + y, took[ft]) / len(took[ft]) )+"\t" )
		f_out.write( ft )
		f_out.write("\n")
		i+=1
	f_out.close()
	print "written timing output"

	print "frames ",len(frames)

	names=[n for n in frames[0]]

	f_out = file(fn_output+".csv", "a")
	for name in names:
		print name
		dist=[]
		gtp=0
		gfp=0
		ges=0
		for i in xrange(len(frames)):
			ges+=len(frames[i][name])
			for ii in xrange(len(frames[i][name])):
				kp1 = frames[i][name][ii]
				for j in range(i+1,len(frames)):
					tp=0
					fp=0
					mi=radius
					for jj in xrange(len(frames[j][name])):
						kp2 = frames[j][name][jj]

						L2=(kp1["pt"]-kp2["pt"]).length()
						if L2<=radius/4:
							if tp>0:
								fp+=1
							else:
								tp+=1
							mi = min(mi, L2)
					if tp>0: dist.append(mi)
					gtp+=tp
					gfp+=fp
		print ""

		aa,bb,cc = mquantiles(dist)

		f_out.write( str(name)+"\t" )
		f_out.write( str(gtp)+"\t" )
		f_out.write( str(gfp)+"\t" )
		f_out.write( str(ges)+"\t" )
		f_out.write( str(len(frames))+"\t" )
		f_out.write( str(min(dist))+"\t")
		f_out.write( str(aa)+"\t" )
		f_out.write( str(bb)+"\t" )
		f_out.write( str(cc)+"\t" )
		f_out.write( str(max(dist))+"\t" )
		f_out.write( str(reduce(lambda x, y: x + y, dist) / len(dist) )+"\t" )
		f_out.write( str(max(dist))+"\n" )
		f_out.flush()

	assert(tfs_file==fts_file)

run()
