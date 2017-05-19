#!/usr/bin/python

from qual import qual_matrix, time_matrix
from plot import line_plot

import sys

#algos=["FSHD","SHOT","VFH","ESF","FPFH"]
algos=["FSHD"]
date=sys.argv[2]
files=sys.argv[3:]
params=[2,4,8,16,32,64,128,256]


gnuplotT=line_plot(sys.argv[1]+"-RADII-timing")
gnuplotT.set_labels("radii", "exec. time [s]")
gnuplotT.set_logscale('x',2)

gnuplot=line_plot(sys.argv[1]+"-RADII")
gnuplot.set_labels("radii", "descriptor distance [L2]", "dim. of feature")
gnuplot.set_range('y',0.5,0.8)
gnuplot.set_logscale('x',2)
gnuplot.set_logscale('x2',2)
gnuplot.add_user('set grid x y2')
gnuplot.add_eq('x*32*32 axes x2y2')
for ft in algos:
	D={}
	T={}
	for param in params:
		print "Param: ",param
		subset_files=[]
		for f in files:
			subset_files.append(f+"-timing-"+str(param)+"-32-"+date+"-eval.csv")
		R=qual_matrix(subset_files,ft,3)
		if len(R)<1: continue
		i=min(R, key=R.get)
		D[param] = R[i]
		T[param] = time_matrix(subset_files)
	gnuplot.add_data(D, ft, 'linespoints')
	gnuplotT.add_data(T, ft)
	print T
gnuplot.add_attr('lc rgb "black')
gnuplot.close()

gnuplotT.close()
print "----------------------DONE STAGE 1------------------------------------"
 
gnuplotT=line_plot(sys.argv[1]+"-ANGLES-timing")
gnuplotT.set_labels("angles", "exec. time [s]")
gnuplotT.set_logscale('x',2)

gnuplot=line_plot(sys.argv[1]+"-ANGLES")
gnuplot.set_labels("angles", "descriptor distance [L2]", "dim. of feature")
gnuplot.set_range('y',0,1)
gnuplot.set_logscale('x',2)
gnuplot.set_logscale('x2',2)
gnuplot.add_user('set grid x y2')
gnuplot.add_eq('x*x*8 axes x2y2')
for ft in algos:
	D={}
	T={}
	for param in params:
		subset_files=[]
		for f in files:
			subset_files.append(f+"-timing-8-"+str(param)+"-"+date+"-eval.csv")
		R=qual_matrix(subset_files,ft,3)
		if len(R)<1: continue
		i=min(R, key=R.get)
		D[param] = R[i]
		T[param] = time_matrix(subset_files)
	gnuplot.add_data(D, ft, 'linespoints')
	gnuplotT.add_data(T, ft)
gnuplot.add_attr('lc rgb "black')
gnuplot.close()

gnuplotT.close()
