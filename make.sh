#!/bin/sh

rm -f imu_logger
gcc -std=gnu99 -pthread -o imu_logger *.c -lm
