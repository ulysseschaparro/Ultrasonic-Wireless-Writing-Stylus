# CSE4342 Embedded Systems II Final Project

# Overview
This project integrates the Tiva™ TM4C123GH6PM Microcontroller with hardware to design a 
wireless writing stylus that can be used for data input. Ultrasonic and IR emitters are used to 
send the position of the stylus to multiple fixed receivers. The ultimate goal is to read a (x,y) 
position of the stylus once the button on the pen is clicked. There is a command line interface 
that is capable of controlling the system and reading the (x,y) position of the stylus. 

# Commands
The project supports the functionality of the following shell commands: 

## reset
Hardware should reset. 

## sensor S, X, Y
Updates and stores the (x,y) location of sensor S in EEPROM. 

## correction S, K0, K1
Updates and stores the 0th and 1st order distance correction factors for sensor S in EEPROM. 

## distance
Displays the distance of the stylus from each of the sensors as long as the push button is pressed.

## method METHOD
Updates and stores the current measurement method in EEPROM. METHOD can be either 2SE2C (two systems of equations of 2 circles) or 2TRI (2 systems of equations of right triangles). 

## average N
Updates N so that the (x,y) reports are the average over at least N (x,y) 
samples. 

## variance V
Updates V so that the upper limit on the variance of the Euclidean distance within a set of N measurements must be less than V for a (x,y) fix to be valid. 

## beep ir ON|OFF
Requests that a beep should be output once a valid IR signal is received if enabled. 

## beep error 2|3 ON|OF
Requests that a beep should be output if either 2 or 3 ultrasonic signals are not received within a timeout period after a valid IR signal is received if enabled. 

## beep 2 ON|OFF
Requests that a beep should be output once a valid IR signal and two ultrasonic signals are received if enabled. 

## beep 3 ON|OFF
Requests that a beep should be output once a valid IR signal and three ultrasonic signals are received if enabled.

## beep fix ON|OFF
Requests that a beep should be output once a valid (x,y) fix is determined if enabled. 

## beep ir|error|2|3|fix freq duration
The beep frequency and duration in us for the selected event is updated and stored in EEPROM. 
