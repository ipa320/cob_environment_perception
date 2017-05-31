#!/usr/bin/python

import csv, tempfile
import os, sys, math

class cfile(file):
    #subclass file to have a more convienient use of writeline
    def __init__(self, name, mode = 'r'):
        self = file.__init__(self, name, mode)

    def wl(self, string):
        self.writelines(string + '\n')
        return None
        
class line_plot:
	def __init__(self, fn):
		self.fn=fn
		self.gnuplot=cfile(fn,"w")
		self.mem=[]
		self.first=True
		
		self.gnuplot.wl("set bars 2.0")
		self.gnuplot.wl("set style fill empty")
		self.gnuplot.wl("set terminal pdf")
		self.gnuplot.wl('set out "'+fn+'-feature.pdf"')
		
	def set_labels(self, xlabel, ylabel=None, y2label=None):
		self.gnuplot.wl('set xlabel "'+xlabel+'"')
		if ylabel!=None: self.gnuplot.wl('set ylabel "'+ylabel+'"')
		if y2label!=None: self.gnuplot.wl('set y2label "'+y2label+'"')
		
	def set_tics(self, xtics, ytics=None, y2tics=None):
		self.gnuplot.wl('set xtics "'+str(xtics)+'"')
		if ytics!=None: self.gnuplot.wl('set ytics "'+str(ytics)+'"')
		if y2tics!=None: self.gnuplot.wl('set y2tics "'+str(y2tics)+'"')
		
	def set_range(self, axis, low, up):
		self.gnuplot.wl('set '+axis+'range ['+str(low)+':'+str(up)+']')
		
	def set_logscale(self, axis='x', base=None):
		if base==None:
			self.gnuplot.wl('set logscale '+axis)
		else:
			self.gnuplot.wl('set logscale '+axis+' '+str(base))
		
	def add_user(self, s):
		self.gnuplot.wl(s)
		
	def add_eq(self, eq):
		if not self.first: self.gnuplot.write(", ")
		else:
			self.gnuplot.write("plot ")
			self.first=False
		self.gnuplot.write(eq)
		
	def add_data(self, R, title, style='lines'):
		if len(R)<1:
			print "WARNING: data empty for "+title
			return
		tmp = tempfile.NamedTemporaryFile(delete=False)
		for i in sorted(R):
			tmp.write(str(i)+"\t"+str(R[i])+"\n")
		tmp.close()
		self.mem.append(tmp.name)
		
		self.add_eq('"'+tmp.name+'" using 1:2 title "'+title+'" with '+style)
		
	def add_data_matrix(self, R, title, i1, i2, style='lines'):
		if len(R)<1:
			print "WARNING: data empty for "+title
			return
		tmp = tempfile.NamedTemporaryFile(delete=False)
		for i in xrange(len(R)):
			tmp.write(str(R[i][i1])+"\t"+str(R[i][i2])+"\n")
		tmp.close()
		self.mem.append(tmp.name)
		
		self.add_eq('"'+tmp.name+'" using 1:2 title "'+title+'" with '+style)
		
	def add_attr(self, s):
		self.gnuplot.write(' '+s)
		
	def close(self):
		self.gnuplot.wl('')
		self.gnuplot.close()
		os.system('gnuplot "'+self.fn+'"')

		for m in self.mem: os.unlink(m)

'''
#
# $Id: multiaxis.dem,v 1.1 2007/06/09 22:10:45 sfeam Exp $
#

# Use the 3rd plot of the electronics demo to show off
# the use of multiple x and y axes in the same plot.
# 
A(jw) = ({0,1}*jw/({0,1}*jw+p1)) * (1/(1+{0,1}*jw/p2))
p1 = 10
p2 = 10000
set dummy jw
set grid x y2
set key center top title " "
set logscale xy
set log x2
unset log y2
set title "Transistor Amplitude and Phase Frequency Response"
set xlabel "jw (radians)"
set xrange [1.1 : 90000.0]
set x2range [1.1 : 90000.0]
set ylabel "magnitude of A(jw)"
set y2label "Phase of A(jw) (degrees)"
set ytics nomirror
set y2tics
set tics out
set autoscale  y
set autoscale y2
plot abs(A(jw)) axes x1y1, 180./pi*arg(A(jw)) axes x2y2


set xrange [-10:10]
set ytics 10 nomirror tc lt 1
set ylabel '2*x' tc lt 1
set y2tics 20 nomirror tc lt 2
set y2label '4*x' tc lt 2
plot 2*x linetype 1, 4*x linetype 2 axes x1y2
'''
