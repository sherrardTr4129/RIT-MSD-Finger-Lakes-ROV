# ROV Microcontroller Teensy Source
This directory contains the arduino sketch used to control data aquisition from sensors on the ROV via digital interfaces to a Teensy 3.2 microcontroller. This Teensy also recieves motor commands from upstream software and actuates the motors accordingly. Communication between the ROV Rasperry Pi 4 and the Teensy is managed through ROS-serial. 

## ROS Topics Used Here
A list of ROS topics, their handle names and their structure can be seen below.

| Topic               | Handle Name   | Message Structure                                                   | Units of Data (If Applicable)                                     |
| :-----------------: | :-----------: | :-----------------------------------------------------------------: | :---------------------------------------------------------------: |
| Float32MultiArray   | imu\_data     | data[xAccel, yAccel, zAccel, xMag, yMag, zMag, xGyro, yGyro, zGyro] | accelerations: m/(s^2), magnetic field: Gauss, gryo data: deg/sec |
