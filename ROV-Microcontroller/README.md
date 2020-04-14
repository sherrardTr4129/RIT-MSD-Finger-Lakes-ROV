# ROV Microcontroller Teensy Source
This directory contains the arduino sketch used to control data aquisition from sensors on the ROV via digital interfaces to a Teensy 3.2 microcontroller. This Teensy also recieves motor commands from upstream software and actuates the motors accordingly. Communication between the ROV Rasperry Pi 4 and the Teensy is managed through ROS-serial. 

## ROS Topics Used Here
A list of ROS topics, their handle names and their structure can be seen below.

| Topic               | Pub/Sub    | Handle Name           | Message Structure or Description                                                               | Units of Data (If Applicable)                                     |
| :-----------------: | :--------: | :-------------------: | :--------------------------------------------------------------------------------------------: | :---------------------------------------------------------------: |
| Float32MultiArray   | Published  | imu\_data             | data[xAccel, yAccel, zAccel, xMag, yMag, zMag, xGyro, yGyro, zGyro]                            | accelerations: m/(s^2), magnetic field: Gauss, gryo data: deg/sec |
| Float32MultiArray   | Published  | BR\_data              | data[pressure, temperature, depth]                                                             | pressure: mBar, temperature: degrees C, depth: meters             |
| Int8                | Subscribed | VertMotionSpeed       | data: Int8 (integer between -2 and 2 indicating direction and speed for vertical ROV motion)   | N.A                                                               |
| Int8                | Subscribed | HorizontalMotionSpeed | data: Int8 (integer between -2 and 2 indicating direction and speed for horizontal ROV motion) | N.A                                                               |
| Int8                | Subscribed | LightSetting          | data: Int8 (integer between 0 and 3 indication the current light controller setting)           | N.A                                                               |
| Int8                | Subscribed | rotationSpeed         | data: Int8 (integer between -2 and 2 indicating direction and speed for rotational ROV motion) | N.A                                                               |
| Int8                | Subscribed | cruiseEnabled         | data: Int8 (integer set as either 0 or 1 indicating the state of cruise control)               | N.A                                                               |

