# abrs_rack
ABRS Team 1

This repository contains the complete software package for the ABRS Capstone project.

KIOSK:
The kiosk software can be built with

$ gcc -o kiosk_interface kiosk_interface.c -l bcm2835
$ gcc -o kiosk_io kiosk_io.c -l bcm2835

and run with

$ sudo ./kiosk_interface
$ sudo ./kiosk_io


RACK:
The rack software is an Arduino Sketch, and is intended to be uploaded to the rack controller using the Arduino IDE.

The settings are

Board -> Arduino Mega2560
Programmer -> Arduino as ISP
