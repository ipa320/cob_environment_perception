#!/usr/bin/python

from qual import qual_matrix
from plot import line_plot

import sys

algos=["FSHD","SHOT","VFH","ESF","FPFH"]
files2=sys.argv[2:]
files=[]
for f in files2:
	if f.find("freiburg2_desk")==-1 and f.find("freiburg2_large_no_loop")==-1:
	#if True or f.find("pioneer")!=-1:
		files.append(f)

gnuplot=line_plot(sys.argv[1])
gnuplot.set_labels("search radius [m]", "avg. deviation [m]")
gnuplot.set_tics(0.1)
gnuplot.add_user('set grid x y')

for ft in algos:
	R=qual_matrix(files,ft)
	gnuplot.add_data(R, ft, 'linespoints')
	
gnuplot.add_attr('lc rgb "black')	
gnuplot.close()
