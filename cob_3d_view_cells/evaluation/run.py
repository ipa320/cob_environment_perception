#!/usr/bin/env python

import sys, subprocess, time
from os import listdir, mkdir, system, getcwd
from os.path import isfile, join, isdir
import shutil, inspect, os

def evaluate(fn,thr1,thr2):
	cur = os.path.dirname(os.path.abspath(sys.argv[0]))
	base= os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
	tmp = base+"/eval_tmp.launch"
	system("cat "+base+"/l1 >"+tmp)
	
	system("echo -n 'args=\"--bag "+getcwd()+"/"+fn+" --bag_out "+getcwd()+"/cap_"+fn+"__"+str(thr1)+"_"+str(thr2)+".bag\"'>>"+tmp)
	system("cat "+base+"/l2 >>"+tmp)
	
	system("echo -n "+str(thr1)+">>"+tmp)
	system("cat "+base+"/l3 >>"+tmp)
	system("echo -n "+str(thr2)+">>"+tmp)
	system("cat "+base+"/l4 >>"+tmp)
	print "cap_"+fn+"__"+str(thr1)+"_"+str(thr2)

	p_view = subprocess.Popen(["roslaunch","cob_3d_view_cells","eval_tmp.launch"], stderr=subprocess.PIPE)
	#p_record = subprocess.Popen(["rosbag","record","-O","cap_"+fn+"__"+str(thr1)+"_"+str(thr2),"-j","-a"])
	#p_launch = subprocess.Popen(["rosbag","play","-r","5",fn], stderr=subprocess.PIPE)
	
	while p_view.poll() is None:
		time.sleep(0.1)
		
	#p_view.kill()
	#p_record.kill()
	#system("killall record")

for fn in sys.argv[1:]:
	print fn
	for thr1 in [60,70]:#[50,60,70,80,90]:#[60]:#:#range(30,101,20):#70
		for thr2 in [4,6]:#[2,3,4,5,6,10]:#[4]:#:#range(3,30,4):#3,16
			evaluate(fn,thr1,thr2/100.)
			print "Settings: ",thr1,thr2
