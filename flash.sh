#!/bin/bash
../resetall.sh
for pico in {0..3}
do
    echo "========= FLASH $pico"
    openocd -f /home/pi/configs/raspberrypi-swd-$pico.cfg -f target/rp2040.cfg -c "program $1 reset exit"
    if [ $? -ne 0 ] ; then
        exit
    fi
done
../resetall.sh
