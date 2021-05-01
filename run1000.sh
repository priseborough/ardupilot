#!/bin/bash

for i in $(seq 1 1000); do
    echo "Starting run $i"
    DAL_SEED=$i DAL_RANDOM_SCALE=0.00000001 ./build/sitl/tools/Replay InertialLogs/SITL-no-noise-test1.BIN
done
