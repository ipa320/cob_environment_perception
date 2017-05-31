#!/usr/bin/python

import os, sys, math
from cgkit.cgtypes import vec3, vec4, quat
from scipy.stats.mstats import mquantiles
def calc_pr(ar, dist=0.35, resolution=100):

	Ts=[]
	_cur=[]
	for a in ar: _cur.append(a[0])
	_cur.sort()
	mi = min(list(_cur))
	ma = max(list(_cur))
	for x in xrange(resolution):
		#Ts.append((ma-mi)*x/(resolution-1)+mi)
		Ts.append(_cur[x*(len(_cur)-1)/(resolution-1)])
		
	R=[]
	for T in Ts:
		_TP=_FN=_FP=_TN=0
		
		for a in ar:
			if a[1]<=dist: #golden std. -> pos
				if a[0]<T: #true
					_TP+=1
				else: #neg
					_FN+=1
			else: #golden std. -> neg
				if a[0]<T: #true
					_FP+=1
				else: #neg
					_TN+=1
					
		#print _TP, _FN, _FP, _TN
					
		try:
			precision = _TP/float(_TP+_FP)
			if _TP==0: precision=1.
			recall = _TP/float(_TP+_FN)
			fpr = _FP/float(_FP+_TN)
			tnr = _TN/float(_TN+_FP)
			acc = (_TP+_TN)/float(_TN+_FP+_TP+_FN)
			
			R.append([T,precision,recall,fpr,tnr,acc])
		except:
			#print "WARN: some error...", sys.exc_info()[0]
			R.append([T,0,0,0,0,0, 0,0,0,0])
	return R
		

num_radii=1
num_angles=1

def dist(ft1, ft2, t):
	global num_angles, num_radii
	assert(len(ft1)==len(ft2))
	r=0
	if t=="FSHD":
		nr = num_radii/2
		for i in xrange(len(ft1)):
			if ft1[(i/nr)*nr]!=0 and ft2[(i/nr)*nr]!=0:
				r += pow((ft1[i]/ft1[(i/nr)*nr]-ft2[i]/ft2[(i/nr)*nr]),2)
			if ft1[0]!=0 and ft2[0]!=0:
				r += pow((ft1[i]/ft1[0]-ft2[i]/ft2[0]),2)
		if r==0: return 1000000000.
	else:
		for i in xrange(len(ft1)):
			r += pow((ft1[i]-ft2[i]),2)
	return math.sqrt(r)

assert(len(sys.argv)==4)

#@profile
def run():
	global num_radii,num_angles
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
	took={}
	for l in f_fts:
		s = l.split(DL)
		if len(s)>0: s[len(s)-1] = s[len(s)-1][0:len(s[len(s)-1])-1]

		if len(s)>0 and s[0]=="file": fts_file = s[1]
		if len(s)>0 and s[0]=="num_radii": num_radii = int(s[1])
		if len(s)>0 and s[0]=="num_angles": num_angles = int(s[1])
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
		if len(s)>0 and s[0]=="took":
			if not s[1] in took: took[s[1]] = []
			took[s[1]].append(reduce(lambda x, y: float(x) + float(y), s[2:]))

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
	avgDBG={}
	for i in xrange(len(keypoints)):
		if len(keypoints)>1000:
			if (i)%(len(keypoints)/1000)==0:
				print "#",
				sys.stdout.flush()
				if i>10: break
		#if i==3: exit()

		fts=[]
		md={}
		mL={}
		mF={}
		mdDBG={}
		mLDBG={}
		mFDBG={}
		kp1 = keypoints[i]
		NNNN=0
		for j in range(i+1,len(keypoints)):
			kp2 = keypoints[j]

			L=(kp1["pt"]-kp2["pt"]).length()
			#print L,"\t",
			for ft in kp1:
				if ft=="pt" or not ft in kp2: continue
				d=dist(kp1[ft], kp2[ft], ft)
				
				if not ft in md:
					md[ft]=None
					mL[ft]=None
					mdDBG[ft]=None
					mLDBG[ft]=None
					fts.append(ft)
				if md[ft]==None or d<md[ft]:
					md[ft] = d
					mL[ft] = L
					mF[ft] = [kp1[ft],kp2[ft]]
				if mdDBG[ft]==None or L<mLDBG[ft]:
					mdDBG[ft] = d
					mLDBG[ft] = L
					mFDBG[ft] = [kp1[ft],kp2[ft]]
			
			'''for ft in kp1:
				if ft=="pt" or not ft in kp2: continue
				d=dist(kp1[ft], kp2[ft], ft)
				if d<=mdDBG[ft]:
					NNNN+=1
					print L'''
		for ft in fts:		
			if md[ft]==None: continue
			
			if not ft in avg:
				avg[ft]=[]
				avgDBG[ft]=[]
				
			avg[ft].append([md[ft],mL[ft],mF[ft]])
			avgDBG[ft].append([mdDBG[ft],mLDBG[ft],mFDBG[ft]])
			
			print ft, NNNN
			print md[ft],mL[ft]
			print mdDBG[ft],mLDBG[ft]
			print mFDBG[ft][0]
			print mFDBG[ft][1]
			print mF[ft][1]

	print "frames ",frames
	print avg
	print avgDBG
	
	for b in xrange(len(borders)):
		for ft in avg:
			f1 = file(fn_output+"_"+ft+"_pr_"+str(borders[b])+".csv", "w")
			R = calc_pr(avg[ft],borders[b])
			for r in R:
				f1.write( str(r[0]) )
				for i in range(1,len(r)):
					f1.write( "\t"+str(r[i]) )
				f1.write("\n")
			f1.close()
		
	assert(tfs_file==fts_file)

run()
