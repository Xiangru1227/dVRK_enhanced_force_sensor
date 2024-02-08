# **Enhanced Modular 3-DoF Force Sensor for dVRK**

This repository houses the design and implementation details of a modular 3-Degree-of-Freedom (3-DoF) force sensor developed for Robot-assisted Minimally Invasive Surgery (RMIS) research. The sensor facilitates effective force modulation during tissue manipulation, ensuring safety in RMIS by providing high-quality, bimanual 3-DoF force data.

## CAD
Includes the *.step files of the sensor base, force plate with rod, insert layer and static calibration plate. Gerber files and CAM for fPCB manufacture.

## Arduino
Program for data acquisition via microcontroller in static calibration.

## static_cal(C++)
Program for collecting and printing data from the force sensor and Nano17 in static calibration.

## Calibration(MatLab)
Program for calculating the calibration matrix for the force sensor.

## ROS
Arduino programmed into microcontroller for data acquisition via PySerial in ROS.
ROS nodes for listening serial port data and printing it to terminal.
