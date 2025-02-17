close all; clear all;


% Connect to Arduino Mega
a = arduino('COM6', 'Mega2560');

% Create an instance of the arm_motor class with the appropriate pin
motor = arm_motor(a, 'D7');

motor.motor_pulse(0.5);
