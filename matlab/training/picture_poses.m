clear all; close all; clc;
addpath('./util/')

% Create an instance of the arm and motor
motor = armMotor();
arm = robotArm();

%% Minimal compression

comp = load('./state/comp.mat').comp;
arm.set_pos(comp)


%% Maximum compression

home = load('./state/home').home;
delta_l = -90;
comp_delta = repmat(delta_l.*[1,5/7,3/7],[1,3]);
max_comp = home + comp_delta;
arm.set_pos(max_comp);


%% Max rotation

comp = load('./state/comp.mat').comp;

dr = -60; % limit for rotation
dr_wrist = -80;
dl = -30; % limit for extension

r_ratio = [1,1,0];
% r_ratio_top = [1,0,0];
r_ratio_top = [1,1,0];
% r_ratio_top = [0,1,.65];

dl_wrist = r_ratio_top.*dr_wrist;
dl_elbow = r_ratio.*dr;
dl_shoulder = r_ratio.*dr;

dl_offset = (max(abs(dl_wrist)) + max(abs(dl_elbow)) + max(abs(dl_shoulder)))/8;
dl = dl + dl_offset;

% Anti-slackening compensation
dl_elbow = dl_elbow + dl_shoulder;
dl_wrist = dl_wrist + 0.75.*dl_elbow;

% Add in compression values
dl_wrist = dl_wrist + dl;
dl_elbow = dl_elbow + (5/7).*dl;
dl_shoulder = dl_shoulder + (3/7).*dl;

rot_arm = [dl_wrist(1),dl_elbow(1),dl_shoulder(1),...
                dl_wrist(2),dl_elbow(2),dl_shoulder(2),...
                dl_wrist(3),dl_elbow(3),dl_shoulder(3)];
rot_comp = rot_arm + comp;

arm.set_pos(rot_comp);