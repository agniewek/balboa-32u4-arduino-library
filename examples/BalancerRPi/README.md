# Balboa32U4 with RPi Balancer example

## Summary

This is an example based on the [Balancer example from the Balboa32U4
library](https://github.com/pololu/balboa-32u4-arduino-library/tree/master/examples/Balancer).
It is regulated to work with the additional load -- Raspberry Pi 3+ and bumpers.

Additionally, the interface to control the robot over UART is provided.

## UART interface
Several commands can be sent to the Balboa robot over UART from Raspberry Pi.
The commands should be separated by the new line '\n'. The possible commands are
listed below:

* 'S' -- Stand up
* 'L' -- Lay down
* 'angular_velocity linear_velocity' -- angular velocity (in 1/100 rad/s) and
linear velocity (in cm/s) with which the robot should move. If the new command
will not be received, robot will automatically stop after 1s.


