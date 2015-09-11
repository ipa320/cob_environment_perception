#!/usr/bin/python

import csv, sys

with open(sys.argv[1], 'rb') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=';', quotechar='|')

    num = 0
    time= 0
    corr= 0
    reloc= 0
    corr_reloc= 0

    biggest_id = 1
    last_id = 1
    
    for row in spamreader:
        num+=1
        time+=float(row[0])
        c=0
        if int(row[1])==1: c=1
        corr+=c

        i = int(row[2])

        if i<=biggest_id and i!=last_id:
            reloc+=1
            corr_reloc+=c

        last_id = i
        biggest_id = max(biggest_id, i)

    print ';'.join([str(time/num), str(corr/float(num)), str(reloc), str(corr_reloc), str(num)])
