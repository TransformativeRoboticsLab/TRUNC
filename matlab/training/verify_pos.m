clear all; close all; clc;
addpath('./util/')

% Create an instance of the arm and motor
motor = armMotor();
arm = robotArm();
arm.reset_arm()

%% Verify

comp = load("./state/comp.mat").comp

points = load("./trajectory/training_waypoints_matrix.mat").points;

pos = points(35975+1,:)

arm.set_pos(comp + pos)

pause(1);

arm.get_pose()