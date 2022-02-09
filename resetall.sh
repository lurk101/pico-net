#!/bin/bash
GPIO=26
SYS=/sys/class/gpio
cd $SYS
echo $GPIO > export
sudo bash -c "echo out   > gpio$GPIO/direction"
sudo bash -c "echo 0     > gpio$GPIO/value"
sleep 0.1
sudo bash -c "echo 1     > gpio$GPIO/value"
echo $GPIO > unexport
