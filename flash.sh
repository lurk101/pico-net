#!/bin/bash
for pico in {0..3}
do
openocd -f /home/pi/configs/raspberrypi-swd-$pico.cfg -f target/rp2040.cfg -c "program $1 reset exit"
done
