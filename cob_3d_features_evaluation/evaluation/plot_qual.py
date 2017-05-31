#!/usr/bin/python

from qual import qual_matrix
from plot import line_plot

import sys

algos=["FSHD","SHOT","VFH","ESF","FPFH"]
files2=sys.argv[2:]
files=[]
for f in files2:
	#if f.find("freiburg2_desk")==-1 and f.find("freiburg2_large_no_loop")==-1:
	if f.find("pioneer")==-1:
		files.append(f)

cols=[[6,"avg. deviation [m]"],[3,"med. deviation [m]"],[7,"precision"],[9,"recall"],[11,"true negative rate"],[13,"accuracy"]]

for C in cols:
	gnuplot=line_plot(sys.argv[1]+"_"+str(C[0]))
	gnuplot.set_labels("search radius [m]", C[1])
	gnuplot.set_tics(0.1)
	gnuplot.add_user('set grid x y')

	for ft in algos:
		R=qual_matrix(files,ft,C[0])
		gnuplot.add_data(R, ft, 'linespoints')
		
	gnuplot.add_attr('lc rgb "black')	
	gnuplot.close()


for dist in [ [0.3,2,1], [0.25,2,1], [0.35,2,1], [0.7,2,1], [0.8,2,1] ]:
	gnuplot=line_plot(sys.argv[1]+"_pr_"+str(dist[0]))
	gnuplot.set_labels("search radius [m]", "re")
	gnuplot.set_tics(0.1)
	gnuplot.add_user('set grid x y')
	#gnuplot.set_range('x', 0, 1)

	for ft in algos:
		R=qual_matrix(files,ft,-1,dist[0])
		print R[0:4]
		gnuplot.add_data_matrix(R, ft, dist[1], dist[2], 'linespoints')
		
	gnuplot.add_attr('lc rgb "black')	
	gnuplot.close()
