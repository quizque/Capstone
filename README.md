# Stargazer Flight Logger
## Capstone 2024

### Summery

Stargazer is an all-in-one logger capable of high-speed data saving and transmitting of high-altitude rocket flights. It is one IO pin away from a full flight computer that is capable of controlling ignition charges and performing TVC. Stargazer contains an IMU (gyroscope/accelerometer), high-g accelerometer, barometer, magnetometer, and GPS. All of these sensors combine together allow for the calculation of direction, trajectory, orientation, etc. which can potentially allow for real-time correction.

### Directory Setup

**sensor_testing/**

Contains programs and results from various sensor tests.

**stargazer/**
 
Contains KiCAD project for Stargazer

**stargazer-demo-src/**

Contains firmware for the Stargazer demo

**stargazer-src/**

Contains the main firmware for Stargazer

**tools/**

Contains the script to convert raw data obtained from Stargazer into a CSV file. Also contains the demo day visualizers

**vibration-bay-src/**

Contains the firmware used in the vibration testing bay