#!/bin/sh

BIN_PATH="."
BIN_NAME="imu_logger"

$BIN_PATH"/"$BIN_NAME $(cat UID.txt)
