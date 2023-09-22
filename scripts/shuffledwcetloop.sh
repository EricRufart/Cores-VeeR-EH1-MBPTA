#!/bin/bash


while :
do
    for i in sources/*
    do
        d=$(basename $i)
        shuf $i --output $d
        #cp $i $d
        cp $d inputfile.txt
        R --no-save < MBPTA-CV.withlicense.R
        cp output.txt $d.out
        cp pwcet.png pwcet"$d".png
        cp cvplot.png cvplot"$d".png
        rm inputfile.txt output.txt pwcet.png cvplot.png
    done
    a=$(grep -r --exclude=*.{R,sh} "pWCET" | wc -l)
    if [ ! $a == 0 ]; then
        break
    fi
done
grep -r  --exclude=*.{R,sh} CONVERGENCE
grep -r  --exclude=*.{R,sh} "TEST FAILED"
grep -r  --exclude=*.{R,sh} pWCET
