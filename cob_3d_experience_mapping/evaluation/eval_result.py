#!/usr/bin/python

import csv, sys, os

with open(sys.argv[1], 'rb') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=';', quotechar='|')

    num = 0
    time= 0
    corr= 0
    corr2= 0
    reloc= 0
    corr_reloc= 0
    corr_reloc2= 0

    biggest_id = 1
    last_id = 1
    
    for row in spamreader:
        num+=1
        time+=float(row[0])
        
        c=0
        if int(row[1])==1: c=1
        corr+=c
        
        x1 = float(row[3])
        y1 = float(row[4])
        x2 = float(row[6])
        y2 = float(row[7])
        dist = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)
        c2=0
        if dist<=0.25: #0.5m
			c2=1
        corr2+=c2

        i = int(row[2])

        if i<=biggest_id and i!=last_id:
            reloc+=1
            corr_reloc+=c
            corr_reloc2+=c2

        last_id = i
        biggest_id = max(biggest_id, i)

	file_size = os.path.getsize(sys.argv[1][:-3]+"map")
    print ';'.join([str(time/num), str(corr/float(num)), str(reloc), str(corr_reloc), str(num), str(corr2/float(num)), str(corr_reloc2), str(file_size)])
