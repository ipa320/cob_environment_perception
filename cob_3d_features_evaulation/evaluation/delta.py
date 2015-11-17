#!/usr/bin/python
import sys

if len(sys.argv)==3:

	f1=open(sys.argv[1], "r")
	f2=open(sys.argv[2], "r")

	l1=f1.read()
	l2=f2.read()

	a1 = l1.split(" ")
	a2 = l2.split(" ")
	assert(len(a1)==len(a2))
	s=0
	s2=0
	for i in range(1,len(a1)/2):
		s+=abs(float(a1[i*2])/float(a1[0])-float(a2[i*2])/float(a2[0]))
		if i%8!=1: s2+=abs(float(a1[i*2])/float(a1[0])-float(a2[i*2])/float(a2[0]))
		print "%6.2f\t"%(float(a1[i*2])/float(a1[0])-float(a2[i*2])/float(a2[0])),
		if (i-1)%8==7: print ""
	print "sum ",s,s2

	f1.close()
	f2.close()
else:
	print "_\t",
	for i in range(1,len(sys.argv)):
		print sys.argv[i],"\t",
	print ""

	for i in range(1,len(sys.argv)):
		f1=open(sys.argv[i], "r")
		l1=f1.read()
		a1 = l1.split(" ")
		print sys.argv[i],"\t",

		for j in range(1,len(sys.argv)):
			f2=open(sys.argv[j], "r")

			l2=f2.read()

			a2 = l2.split(" ")
			assert(len(a1)==len(a2))
			s=0
			for i in range(1,len(a1)/2):
				if i%8!=1: s+=abs(float(a1[i*2])-float(a2[i*2]))
			print s,"\t",

			f2.close()

		f1.close()
		print ""
