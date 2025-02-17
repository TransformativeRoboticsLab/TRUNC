clear all;
addpath('./util');

if ~exist('nnc','var')
    nnc = connect_to_natnet();
end

tool = nnc.getFrame().RigidBodies(1) %End Effector
x = {tool.x, tool.y, tool.z};
q = {tool.qx, tool.qy, tool.qz, tool.qw};

home_pos = load("home_comp.mat").home_comp;

% Connect to servos and initialize the arm
if ~exist('port','var')
    port = serialport("COM4", 9600);
    channels = 0:11;
    pos = zeros(12,1);
    global position;
    if 1
        initialize_servos(port, channels, 1,pos,home_pos);
        pause(5);
        stop_servos(port, channels);
    end
end