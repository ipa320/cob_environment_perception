#!/usr/bin/python

import os, sys, math
from cgkit.cgtypes import vec3, vec4, quat
from scipy.stats.mstats import mquantiles

def dist(ft1, ft2, t):
	assert(len(ft1)==len(ft2))
	r=0
	for i in xrange(len(ft1)):
		#if t=="FSHD" and i%16==0: continue
		#r += abs(ft1[i]-ft2[i])
		r += pow(ft1[i]-ft2[i],2)
	return math.sqrt(r)

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
	keypoints=[]
	KPstart=0
	frames=0
	for l in f_fts:
		s = l.split(DL)
		if len(s)>0: s[len(s)-1] = s[len(s)-1][0:len(s[len(s)-1])-1]

		if len(s)>0 and s[0]=="file": fts_file = s[1]
		if len(s)>0 and s[0]=="header":
			T=None
			ts = float(s[1])
			best=None
			mi=0.075 #minimum
			for t in tfs:
				if abs(float(t[0])-ts)<mi:
					mi = abs(float(t[0])-ts)
					best = t
			if best==None:
				#print "skipping frame as no valid tf was found"
				continue
			#print mi

			T = [vec4(float(best[1]), float(best[2]), float(best[3]), 1.), quat(float(v) for v in best[4:])]
			KPstart=len(keypoints)
			KPtype=""
			frames+=1

		if len(s)==4 and s[0]=="keypoint" and T!=None:
			pt = vec4(float(s[1]), float(s[2]), float(s[3]), 1.)
			TT=T[1].toMat4().setColumn(3, T[0])
			#pt = TT.inverse()*pt
			pt = TT*pt
			keypoints.append({"pt":pt})
			#print pt
		if len(s)>4 and s[0]=="keypoint" and T!=None:
			if KPtype!=s[1]:
				KPtype=s[1]
				KP=KPstart
			keypoints[KP][s[1]] = [float(v) for v in s[2:]]
			KP += 1

	print "read input file ",len(keypoints)

	borders=[x/20. for x in range(1,31)]
	avg={}
	fts=[]
	for i in xrange(len(keypoints)):
		if (i)%(len(keypoints)/1000)==0:
			print "#",
			sys.stdout.flush()
		#if i==3: exit()

		kp1 = keypoints[i]
		for j in range(i+1,len(keypoints)):
			kp2 = keypoints[j]

			L=(kp1["pt"]-kp2["pt"]).length()
			#print L,"\t",
			for ft in kp1:
				if ft=="pt" or not ft in kp2: continue
				#D=dist(kp1[ft], kp2[ft], ft)
				#if D==0:
				#	#print i,j, kp1["pt"], kp2["pt"]
				#	if i+1==j: continue
				#print D,"\t",
				if not ft in avg:
					avg[ft]=0.
					avg[ft+"num"]=0
					avg[ft+"b"]=0.
					avg[ft+"bnum"]=0
					#avg[ft+"ar"]=[]
					#avg[ft+"bar"]=[]
					avg[ft+"border"]=[[] for x in xrange(len(borders))]
					#avg[ft+"bborder"]=[[] for x in xrange(len(borders))]
					fts.append(ft)
				#p=""
				#if L>0.7/3: p="b"
				#avg[ft+p]+=D
				#avg[ft+p+"num"]+=1
				#avg[ft+p+"ar"].append(D)
				D=False
				for b in range(int(L/0.05), len(borders)): #for b in xrange(len(borders)):
					pp=""
					if L>borders[b]: continue #pp="b"
					if D==False: D=dist(kp1[ft], kp2[ft], ft)
					avg[ft+pp+"border"][b].append(D)
			#print ""

	print "frames ",frames
	for ft in fts:
		#if avg[ft+"bnum"]>0 and avg[ft+"bnum"] and avg[ft]>0:
		#	print ft, avg[ft+"num"], avg[ft+"bnum"], avg[ft]/avg[ft+"num"], "  ", avg[ft+"b"]/avg[ft+"bnum"], "   ->  ", (avg[ft+"b"]/avg[ft+"bnum"])/(avg[ft]/avg[ft+"num"])
		#else:
		#	print ft," is emtpy"
		print ft
		#print min(avg[ft+"ar"]),  mquantiles(avg[ft+"ar"]),  max(avg[ft+"ar"])
		#print min(avg[ft+"bar"]), mquantiles(avg[ft+"bar"]), max(avg[ft+"bar"])
		f_out = file(fn_output+"_"+ft+".csv", "w")
		aa,bb,cc = mquantiles(avg[ft+"border"][len(borders)-1])
		FACT = 1/bb
		for pp in [""]:#["", "b"]:
			for b in xrange(len(borders)):
				if len(avg[ft+pp+"border"][b])<1: continue
				f_out.write( str(borders[b])+"\t" )
				aa,bb,cc = mquantiles(avg[ft+pp+"border"][b])
				f_out.write( str(min(avg[ft+pp+"border"][b])*FACT)+"\t")
				f_out.write( str(aa*FACT)+"\t" )
				f_out.write( str(bb*FACT)+"\t" )
				f_out.write( str(cc*FACT)+"\t" )
				f_out.write( str(max(avg[ft+pp+"border"][b])*FACT)+"\t" )
				f_out.write( str(reduce(lambda x, y: x + y, avg[ft+pp+"border"][b]) / len(avg[ft+pp+"border"][b])*FACT ) )
				f_out.write("\n")
			#print ""
		f_out.close()
	assert(tfs_file==fts_file)

run()
