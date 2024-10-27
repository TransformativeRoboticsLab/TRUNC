close all; clear all; clc;
addpath('./util')

% colors
map = brewermap(9,'Set1');

%% Motherboard demo analysis

T = readtable('./data/2024_02_19_21_08_57/positions_norm_full.csv');

ref = load('./inference/motherboard_trajectory.mat').wp;

measured = readtable('./demo/MOTHERBOARD-2024-02-29 01.40.45 PM.csv','VariableNamesRow',4);

x_end_avg = measured.TRUNC_x;
y_end_avg = measured.TRUNC_y;
z_end_avg = measured.TRUNC_z;
qx_end_avg = measured.TRUNC_qx;
qy_end_avg = measured.TRUNC_qz;
qz_end_avg = measured.TRUNC_qy;
qw_end_avg = measured.TRUNC_qw;

T_measured = table(x_end_avg,y_end_avg,z_end_avg,qx_end_avg,qy_end_avg,qz_end_avg,qw_end_avg);
T_measured = rmmissing(T_measured);
T_measured = batch_transform(T,T_measured);


figure(1); clf;
hold on; grid on; axis equal;
set(gcf, 'Color', 'white');

% Replotting for the second tile with X-Z view
plot3(T_measured(:,1),T_measured(:,2),T_measured(:,3),'-','Color',map(1,:),'MarkerSize',8,'LineWidth',2,'DisplayName','DNN');
plot3(ref(:,1),ref(:,2),ref(:,3),'-','Color','black','LineWidth',2,'DisplayName','actual');


%% Lightbulb demo analysis


T = readtable('./data/2024_02_19_21_08_57/positions_norm_full.csv');

ref = load('./inference/lightbulb_trajectory.mat').wp;

measured = readtable('./demo/LIGHTBULB-2024-03-06 11.55.52 AM.csv','VariableNamesRow',4);
measured = measured(measured.Time>=64,:); 

x_end_avg = measured.TRUNC_x;
y_end_avg = -measured.TRUNC_z;
z_end_avg = measured.TRUNC_y;
qx_end_avg = measured.TRUNC_qx;
qy_end_avg = measured.TRUNC_qy;
qz_end_avg = measured.TRUNC_qz;
qw_end_avg = measured.TRUNC_qw;

T_measured = table(x_end_avg,y_end_avg,z_end_avg,qx_end_avg,qy_end_avg,qz_end_avg,qw_end_avg);
T_measured = table2array(T_measured);


figure(1); clf;
hold on; grid on; axis equal;
set(gcf, 'Color', 'white');

% Replotting for the second tile with X-Z view
plot3(T_measured(:,1),T_measured(:,2),T_measured(:,3),'.-','Color',map(1,:),'MarkerSize',8,'LineWidth',2,'DisplayName','DNN');
plot3(ref(:,1),ref(:,2),ref(:,3),'-','Color','black','LineWidth',2,'DisplayName','actual');



%% Valve demo analysis

T = readtable('./data/2024_02_19_21_08_57/positions_norm_full.csv');

ref = load('./inference/valve_trajectory.mat').wp;

measured = readtable('./demo/VALVE-2024-03-10 09.41.13 PM.csv','VariableNamesRow',4);
measured = measured(measured.Time>=90,:); 

x_end_avg = measured.TRUNC_x;
y_end_avg = -measured.TRUNC_z;
z_end_avg = measured.TRUNC_y;
qx_end_avg = measured.TRUNC_qx;
qy_end_avg = measured.TRUNC_qy;
qz_end_avg = measured.TRUNC_qz;
qw_end_avg = measured.TRUNC_qw;

T_measured = table(x_end_avg,y_end_avg,z_end_avg,qx_end_avg,qy_end_avg,qz_end_avg,qw_end_avg);
T_measured = batch_transform(T,T_measured);

figure(1); clf;
hold on; grid on; axis equal;
set(gcf, 'Color', 'white');

% Replotting for the second tile with X-Z view
plot3(T_measured(:,1),T_measured(:,2),T_measured(:,3),'.-','Color',map(1,:),'MarkerSize',8,'LineWidth',2,'DisplayName','DNN');
plot3(ref(:,1),ref(:,2),ref(:,3),'-','Color','black','LineWidth',2,'DisplayName','actual');

% plot_triad(T_measured(:,1),T_measured(:,2),T_measured(:,3),[T_measured(:,7),T_measured(:,4),T_measured(:,5),T_measured(:,6)])

%% Helper functions
% Applies transformation to T2 based on origin pose described by T1
function w_new = batch_transform(T1,T2)
    
    q1 = [T1.qw_end_avg(1),T1.qx_end_avg(1),T1.qy_end_avg(1),T1.qz_end_avg(1)];
    p1 = [T1.x_end_avg(1),T1.y_end_avg(1),T1.z_end_avg(1)];
    

    q2 = [T2.qw_end_avg(1),T2.qx_end_avg(1),T2.qy_end_avg(1),T2.qz_end_avg(1)];
    p2 = [T2.x_end_avg(1),T2.y_end_avg(1),T2.z_end_avg(1)];
    

    w_new = zeros([size(T2,1),7]);

    for i = 1:size(T2,1)
        qi = [T2.qw_end_avg(i),T2.qx_end_avg(i),T2.qy_end_avg(i),T2.qz_end_avg(i)];
        pi = [T2.x_end_avg(i),T2.y_end_avg(i),T2.z_end_avg(i)];
        [qi_new,pi_new] = coordinate_transform(q1,p1,q2,p2,qi,pi);
        w_new(i,1:3) = pi_new.';
        w_new(i,4:end) = [qi_new(2:4),qi_new(1)];
    end

end


function plot_triad(x, y, z, q)
    figure(); clf; hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    arrowLength = 20; % Adjust the arrow length as needed
    headSize = 20;
        

    for i = 1:length(x)
        % Convert quaternion to rotation matrix
        R = quat2rotm(q(i,:));
        
        % Origin for the triad
        origin = [x(i), y(i), z(i)];
        
        % Directions for the triad arrows, transformed by R
        xDir = R(:,1)';
        yDir = R(:,2)';
        zDir = R(:,3)';
        
        % Plot arrows
        quiver3(origin(1), origin(2), origin(3), xDir(1), xDir(2), xDir(3), arrowLength, 'r', 'LineWidth', 2, 'MaxHeadSize', headSize);
        quiver3(origin(1), origin(2), origin(3), yDir(1), yDir(2), yDir(3), arrowLength, 'g', 'LineWidth', 2, 'MaxHeadSize', headSize);
        quiver3(origin(1), origin(2), origin(3), zDir(1), zDir(2), zDir(3), arrowLength, 'b', 'LineWidth', 2, 'MaxHeadSize', headSize);
    end
    
    hold off;
end