#!/bin/bash
#gps script by Mauro Verhoest 6/4/2023

#start a roscore 
gnome-terminal -- bash -c "roscore; exec bash" '$SHELL'

#we wait a little because the roscore must be started before we can run the nmea_serial driver
sleep  5s
gnome-terminal -- bash -c "rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyUSB0  _baud:=115200; exec bash" --role=root

#print out  locatian message
gnome-terminal -- bash -c "rostopic echo /fix"





