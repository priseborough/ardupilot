#!/bin/bash

for i in $(seq 1 100); do
    echo "Starting run $i"
    DAL_SEED=$i ./build/sitl/tools/Replay  --param EK3_IMU_MASK=1 --param EK2_ENABLE=0 /media/psf/Home/Downloads/SITL-no-noise-test1.BIN
done
