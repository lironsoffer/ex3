#!/bin/bash
# Basic until loop
counter=1
until [ $counter -gt 4 ]
do
    python test_checker${counter}.py -> outputs/${counter}
    i=1
    until [ $i -gt 9 ]
    do
        python test_checker${counter}.py >> outputs/${counter}
        ((i++))
    done
    ((counter++))
done
