%% Global setup

close all; clear all; clc
warning('off','all')

% fig scale factor
fig_s = 4;
n  = 10;

% rename width and height to avoid conflict with built-in functions
fig_width = 1.25;
fig_height = 1.25;
view_ax = -150;
view_el = 30;

% fonts
cb_font_size = 4*fig_s;
ax_font_size = 5*fig_s;
ax_label_size = 7*fig_s;
set(0,'DefaultTextFontname', 'CMU Sans Serif' )
set(0,'DefaultAxesFontName', 'CMU Sans Serif' )

% colors
map = brewermap(9,'Set1');

% save path
export_fig = true;
random_order = false;

home_pos = load('./training/state/home_measured.mat').pos;

%% Load in data

if random_order
   T = readtable("./training/repeatability/2024_02_06_11_33_51/positions.csv");
else
    T = readtable("./training/repeatability/2024_02_06_08_53_05/positions.csv");
end

% Flip z and y
n_points = 100;

% Subtract home post
T.x_end_avg = (1000.*T.x_end_avg) - home_pos(1);
T.y_end_avg = (1000.*T.y_end_avg) - home_pos(2);
T.z_end_avg = (1000.*T.z_end_avg) - home_pos(3);


% find distance from mean, averge distance error, and distance variance
end_point = [reshape(T.x_end_avg,n_points,1,[]),reshape(T.y_end_avg,n_points ,1,[]),reshape(T.z_end_avg,n_points ,1,[])];
end_point_mean = mean(end_point,3);
end_point_d = sqrt(sum((end_point-end_point_mean).^2,2));
end_point_mean_e = std(end_point_d,0,3);

if random_order
    end_point_mean = end_point_mean(T.p_idx(1:100),:);
    end_point_mean_full = repmat(end_point_mean,[5,1]);
else
    end_point_mean_full = repmat(end_point_mean,[5,1]);
end

% quaternion distance from mean
quat = [reshape(T.qw_end_avg,n_points,1,[]),reshape(T.qx_end_avg,n_points ,1,[]),reshape(T.qy_end_avg,n_points ,1,[]),reshape(T.qz_end_avg,n_points ,1,[])];
end_point_quat = repmat(quaternion(1, 0, 0, 0), size(quat,1), size(quat,3));

% just matlab conversion from array to object
for ii=1:size(quat,1)
    for jj = 1:size(quat,3)
        end_point_quat(ii,jj) = quaternion(quat(ii,:,jj));
    end
end

% use built in methods to find rotational distances
end_point_quat_mean = meanrot(end_point_quat,2);
end_point_quat_d = rad2deg(dist(end_point_quat,end_point_quat_mean));
end_point_quat_mean_e = std(end_point_quat_d,0,2);

% Display metrics
end_point_d = reshape(end_point_d ,1,[]);
end_point_quat_d = reshape(end_point_quat_d ,1,[]);

disp(mean(end_point_d));
disp(std(end_point_d))
disp(mean(end_point_quat_d));
disp(std(end_point_quat_d))

%% Create Tiled Layout
fig = figure('Units', 'inches', 'Position', [0, 0, 2 * fig_width * fig_s, 2 * fig_height * fig_s]); box on;
tiledlayout(2, 2);
set(gcf, 'color', 'w');

%% Figure 1.1
ax1 = nexttile;
hold(ax1, 'on');
grid(ax1, 'on');
box(ax1, 'on')

% Plot data for Figure 2.1
s = scatter3(ax1, end_point_mean(:,1), end_point_mean(:,2), end_point_mean(:,3), [], end_point_mean_e, 'filled', 'MarkerEdgeColor', "k", 'LineWidth', 1.25);
s.SizeData = 150;

% Set colormap and add color bar for Figure 2.1
colormap(ax1, flipud(brewermap([], 'Spectral')));
cb = colorbar(ax1, 'northoutside', 'LineWidth', 1.5, 'FontSize', cb_font_size);
clim([0,4])
view(ax1, [view_ax, view_el]);

xticks(ax1,[-300,-150,0,150,300])
yticks(ax1,[-300,-150,0,150,300])
xtickangle(ax1, 0)
ytickangle(ax1, 0)

%% Figure 1.2
ax2 = nexttile;
hold(ax2, 'on');
grid(ax2, 'on');
box(ax2, 'on')

% Plot data for Figure 2.2
s = scatter3(ax2, end_point_mean(:,1), end_point_mean(:,2), end_point_mean(:,3), [], end_point_quat_mean_e(1:end), 'filled', 'MarkerEdgeColor', "k", 'LineWidth', 1.25);
s.SizeData = 150;

% Set colormap and add color bar for Figure 2.2
colormap(ax2, flipud(brewermap([], 'RdBu')));
cb = colorbar(ax2, 'northoutside', 'LineWidth', 1.5, 'FontSize', cb_font_size);
clim([0,3])
view(ax2, [view_ax, view_el]);

xticks(ax2,[-300,-150,0,150,300])
yticks(ax2,[-300,-150,0,150,300])
xtickangle(ax2, 0)
ytickangle(ax2, 0)

%% Figure 1.3
ax3 = nexttile;
hold(ax3, 'on'); 
grid on;
box(ax3, 'on')

% Plot data for Figure 2.3
histogram(ax3, reshape(end_point_d,1,[]), 'facecolor', "#FAD1D2", 'FaceAlpha', 1, 'EdgeColor', 'none');
histogram(ax3, reshape(end_point_d,1,[]), 'EdgeColor', map(1,:), 'linewidth', 3, 'DisplayStyle', 'stairs');
xlim([0,13])

%% Figure 1.4
ax4 = nexttile;
hold(ax4, 'on'); 
grid on;
box(ax4, 'on')

% Plot data for Figure 2.4
histogram(ax4, reshape(end_point_quat_d,1,[]), 'facecolor',"#D7E5F1", 'FaceAlpha', 1 ,'EdgeColor', 'none');
histogram(ax4, reshape(end_point_quat_d,1,[]), 'EdgeColor', map(2,:), 'linewidth', 3, 'DisplayStyle', 'stairs');
xlim([0,9])

%% Final Adjustments
set(findobj(gcf, 'type', 'axes'), 'FontSize', ax_font_size, 'LineWidth', 2);

if export_fig
    if random_order
        exportgraphics(gcf,'../figures/repetability/random.emf','ContentType', 'vector');
    else
        exportgraphics(gcf,'../figures/repetability/trajectory.emf','ContentType', 'vector');
    end
end
