#!/usr/bin/python

from qual import qual_matrix
from plot import line_plot

import sys

algos=["FSHD","SHOT","VFH","ESF","FPFH"]
files=sys.argv[2:]

gnuplot.set_labels("search radius [m]", "avg. deviation [m]")
gnuplot.set_tics(0.1)
gnuplot.add_user('set grid x y')

for ft in algos:
	R=qual_matrix(files,ft)
	gnuplot.add_data(R, ft)
	gnuplot.add_attr('with linespoints"')
	
gnuplot.add_attr('lc rgb "black"')	
gnuplot.close()
