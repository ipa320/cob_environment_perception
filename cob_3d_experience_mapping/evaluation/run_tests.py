#!/usr/bin/python

from subprocess import call
import sys
import os.path

file_gparam = open("general_param.xml","r")
params = file_gparam.read()
file_gparam.close()

fn_bagfile=sys.argv[1]
fn_out = "result/eval_em_"
exe = "/home/josh/catkin_ws/devel/lib/cob_3d_experience_mapping/cob_3d_experience_mapping_lemon_eval"

gs_tr=sys.argv[2]
gs_rt=sys.argv[3]
gs_num=sys.argv[4]

def repl(s):
    global thr_tra, thr_rot, dev, max_e, num_cells
    tmp = s.replace("$THR_TRA",str(thr_tra)).replace("$THR_ROT",str(thr_rot)).replace("$ESTO",str(est_occ))
    tmp = tmp.replace("$DEV",str(dev)).replace("$MAXE",str(max_e)).replace("$NUMC",str(num_cells))
    return tmp

for thr_tra in [x * 0.2 for x in range(1, 8)]:
    for thr_rot in [x * 0.2 for x in range(1, 6)]:
        for dev in [0.01, 0.025, 0.05, 0.1, 0.2]:
            for max_e in [1,2,4]:
                for num_cells in [100,500]:
                    for est_occ in [x * 3 for x in range(1, 4)]:
                        suffix = repl("$THR_TRA_$THR_ROT_$DEV_$MAXE_$NUMC_$ESTO")
                        print suffix
                        
                        if os.path.isfile(fn_out+suffix+".map"): #do not repeat
							continue

                        fn_p = fn_out+suffix+".param"
                        f = open(fn_p, "w")
                        f.write(repl(params))
                        f.close()

                        call([exe, "--topic_odom", "/robot0/odom", "--grid_trans", gs_tr, "--grid_rot", gs_rt, "--grid_num", gs_num, "--param", fn_p, "--bag", fn_bagfile, "--result", fn_out+suffix+".csv", "--output", fn_out+suffix+".map"])
