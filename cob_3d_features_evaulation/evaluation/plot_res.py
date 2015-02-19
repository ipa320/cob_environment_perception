#!/usr/bin/python

from qual import qual_matrix
from plot import line_plot

import sys

algos=["FSHD","SHOT","VFH","ESF","FPFH"]
date=sys.argv[2]
files=sys.argv[3:]
params=[2,4,8,16,32,64,128,256]


gnuplot=line_plot(sys.argv[1]+"-RADII")
gnuplot.set_labels("search radius [m]", "avg. deviation [m]", "dim. of feature")
gnuplot.set_tics(0.1)
gnuplot.add_user('set grid x y2')
gnuplot.add_eq('x*32*32 axes x2y2')
for ft in algos:
	D={}
	for param in params:
		subset_files=[]
		for f in files:
			subset_files.append(f+"-timing-"+param+"-32-"+date+"-eval.csv")
		R=qual_matrix(subset_files,ft)
		i=min(R, key=R.get)
		D[param] = R[i]
	gnuplot.add_data(D, ft)
	gnuplot.add_attr('with linespoints"')
gnuplot.add_attr('lc rgb "black"')
gnuplot.close()

 
gnuplot=line_plot(sys.argv[1]+"-ANGLES")
gnuplot.set_labels("search radius [m]", "avg. deviation [m]", "dim. of feature")
gnuplot.set_tics(0.1)
gnuplot.add_user('set grid x y2')
gnuplot.add_eq('x*x*8 axes x2y2')
for ft in algos:
	D={}
	for param in params:
		subset_files=[]
		for f in files:
			subset_files.append(f+"-timing-8-"+param+"-"+date+"-eval.csv")
		R=qual_matrix(subset_files,ft)
		i=min(R, key=R.get)
		D[param] = R[i]
	gnuplot.add_data(D, ft)
	gnuplot.add_attr('with linespoints"')
gnuplot.add_attr('lc rgb "black"')
gnuplot.close()
