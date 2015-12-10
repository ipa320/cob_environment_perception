#!/usr/bin/python

import os, sys, math
from cgkit.cgtypes import vec3, vec4, quat
from scipy.stats.mstats import mquantiles
import rosbag

def calc_pr(_cur,_dif, resolution=100):

	Ts=[]
	_cur.sort()
	mi = min(list(_cur))
	ma = max(list(_cur))
	for x in xrange(resolution):
		#Ts.append((ma-mi)*x/(resolution-1)+mi)
		Ts.append(_cur[x*(len(_cur)-1)/(resolution-1)])
		
	R=[]
	for T in Ts:
		FN = N = 0
		for x in _cur:
			if x<=T: N+=1
		for x in _dif:
			if x>T: FN+=1
			
		_TP=N
		_FN=len(_cur)-N
		
		_FP=len(_dif)-FN
		_TN=FN
			
		try:
			precision = _TP/float(_TP+_FP)
			if _TP==0: precision=1.
			recall = _TP/float(_TP+_FN)
			fpr = _FP/float(_FP+_TN)
			tnr = _TN/float(_TN+_FP)
			acc = (_TP+_TN)/float(_TN+_FP+_TP+_FN)
			
			R.append([T,precision,recall,fpr,tnr,acc, N,FN,len(_cur),len(_dif)])
		except:
			print "WARN: some error...", sys.exc_info()[0]
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
		#for i in xrange(len(ft1)):
		#	if (i/2)%(num_angles/4)==0 and i%2==0: print
		#	print ft1[i],
		#print
		for i in xrange(len(ft1)):
			try:
				r += pow((ft1[i]/ft1[(i/nr)*nr]-ft2[i]/ft2[(i/nr)*nr]),2)
			except:
				pass
			try:
				r += pow((ft1[i]/ft1[0]-ft2[i]/ft2[0]),2)
			except:
				pass
			#if (i)%(num_angles/2)<num_angles/4: r += pow((ft1[i]-ft2[i]),2)
			#r += pow((ft1[i]-ft2[i]),2) *pow((i/2)/(num_angles/4)+1, 1.3)
			#r += abs(ft1[i]-ft2[i])
			#if i%num_angles>num_angles/2: r += pow((ft1[i]-ft2[i]),2)
			#if (i/2)%(num_angles/4)==0 and i%2==0: print
		#	print ft1[i]*pow((i/2)/(num_angles/4)+1, 1.3),
		#print
	else:
		for i in xrange(len(ft1)):
			r += pow((ft1[i]-ft2[i]),2)
		#r += abs(ft1[i]-ft2[i])
		#r += pow((ft1[i]-ft2[i]),2) #*pow(2, i%num_angles)
	return math.sqrt(r)

assert(len(sys.argv)==4)

#@profile
def run():
	global num_radii,num_angles
	f_fts = open(sys.argv[1],'r')
	f_tfs = open(sys.argv[2],'r')
	fn_output = sys.argv[3]
	
	bag = None
	if len(sys.argv)>4: bag = rosbag.Bag(sys.argv[4],'w')

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
			
			if bag!=None:
				pc = sensor_msgs.msg.PointCloud2()
				pc.header = depth_image.header
				pc.width = depth_image.width
				pc.height  = 1
				pc.fields.append(sensor_msgs.msg.PointField(
				name = "x",offset = 0,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
				depth_points.fields.append(sensor_msgs.msg.PointField(
				name = "y",offset = 4,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
				depth_points.fields.append(sensor_msgs.msg.PointField(
				name = "z",offset = 8,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
				depth_points.point_step = 16 
				depth_points.row_step = depth_points.point_step * depth_points.width
				buffer = []
				buffer_rgb = []
				for v in range(depth_image.height):
				for u in range(depth_image.width):
				    d = cv_depth_image[v,u]
				    ptx = (u - centerX) * d / depthFocalLength;
				    pty = (v - centerY) * d / depthFocalLength;
				    ptz = d;
				    buffer.append(struct.pack('ffff',ptx,pty,ptz,1.0))
				depth_points.data = "".join(buffer)
				
				bag.write('/keypoints', pc)
		if len(s)>4 and s[0]=="keypoint" and T!=None:
			vs = [float(v) for v in s[2:]]
			if sum(vs)!=0:
				if KPtype!=s[1]:
					KPtype=s[1]
					KP=KPstart
				keypoints[KP][s[1]] = vs
				KP += 1

	print "read input file ",len(keypoints)
	if bag!=None: bag.close()

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

	borders=[x/20. for x in range(1,31)]
	avg={}
	fts=[]
	for i in xrange(len(keypoints)):
		if len(keypoints)>1000:
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

				_all=avg[ft+"border"][len(borders)-1]
				_cur=avg[ft+pp+"border"][b]
				_dif=set(_all).difference(_cur)
				if len(_dif)<1:
					precision=precisionM=0
					recall=recallM=0
				else:
					T = sum(list(_dif))/len(_dif)
					TM = mquantiles(list(_dif))[0]
					N = 0
					NM= 0
					FN=FNM=0
					for x in _cur:
						if x<=T: N+=1
						if x<=TM: NM+=1
					for x in _dif:
						if x>T: FN+=1
						if x>TM: FNM+=1
						
					recall = N/float(len(_cur))
					recallM = NM/float(len(_cur))
					precision = N/float(N+len(_dif)-FN)
					precisionM = NM/float(NM+len(_dif)-FNM)
					
					tnr = (len(_dif)-FN)/float(len(_dif))
					acc = (len(_dif)-FN+N)/float(len(_all))
					
					tnrM = (len(_dif)-FNM)/float(len(_dif))
					accM = (len(_dif)-FNM+NM)/float(len(_all))
					
				#precision
				f_out.write( "\t"+str(precision) )
				f_out.write( "\t"+str(precisionM) )
				
				#recall
				f_out.write( "\t"+str(recall) )
				f_out.write( "\t"+str(recallM) )
				
				#true negative rate
				f_out.write( "\t"+str(tnr) )
				f_out.write( "\t"+str(tnrM) )
				
				#accuracy
				f_out.write( "\t"+str(acc) )
				f_out.write( "\t"+str(accM) )
				
				#more
				f_out.write( "\t"+str(len(_cur)) )
				f_out.write( "\t"+str(len(_dif)) )
				f_out.write( "\t"+str(N) )
				f_out.write( "\t"+str(len(_dif)-FN) )
				
				#F-measure (IS NOT CORRECT AS FN IS JUST A SUBSET OF ALL!!!!)
				if (recall+precision)>0: f_out.write( "\t"+str(recall*precision*2/(recall+precision)) )
				else: f_out.write( "\t0" )
				if (recallM+precisionM)>0: f_out.write( "\t"+str(recallM*precisionM*2/(recallM+precisionM)) )
				else: f_out.write( "\t0" )
				
				f_out.write("\n")
				
				if borders[b]>=0.2 and borders[b]<=1:
					f1 = file(fn_output+"_"+ft+"_pr_"+str(borders[b])+".csv", "w")
					R = calc_pr(_cur,_dif)
					for r in R:
						f1.write( str(r[0]) )
						for i in range(1,len(r)):
							f1.write( "\t"+str(r[i]) )
						f1.write("\n")
					f1.close()
			#print ""
		f_out.close()
	assert(tfs_file==fts_file)

run()
