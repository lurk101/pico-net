#!/bin/bash
echo "reset halt" > flash.cfg
echo "load $2.elf" >> flash.cfg
echo "reset halt" >> flash.cfg
openocd -f /home/pi/openocd/raspberrypi-swd-$1.cfg -f target/rp2040.cfg -f flash.cfg
