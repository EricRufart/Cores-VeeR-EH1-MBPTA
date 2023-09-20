#!/bin/bash

if [ ! -d "$1" ] || [ ! -d "$2" ]; then
    echo "Path is not directory"
    exit
fi

for d in lh rh rprrDynBP_hash
do
    for i in {1..17} cmark dhry
    do
        cp "$1"/"$d"/$i.txt inputfile.txt
        R --no-save < MBPTA-CV.withlicense.R
        cp output.txt $2/$d/$i.out
        cp pwcet.png $2/$d/pwcet$i.png
        cp cvplot.png $2/$d/cvplot$i.png
        rm inputfile.txt output.txt pwcet.png cvplot.png
    done
done

cd $2
grep -r CONVERGENCE
grep -r "TEST FAILED"
