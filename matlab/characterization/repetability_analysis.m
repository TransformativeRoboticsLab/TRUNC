%% Global setup

close all; clear all; clc
warning('off','all')

% fig sizes and scale factor
fig_w = 300; fig_h = 300; fig_s = 3;

% fonts
ax_font_size = 7*fig_s;
set(0,'DefaultTextFontname', 'CMU Sans Serif' )
set(0,'DefaultAxesFontName', 'CMU Sans Serif' )

% colors
map = brewermap(9,'Set1');

% save path
export_fig = true;


%% Load in data

T = readtable("../Data/full/15000_pose_data_07_26_2023");

% quaternion distance from mean
quat = [T.qx_end_avg,T.qy_end_avg,T.qz_end_avg,T.qw_end_avg];
end_point_quat = quaternion(quat);
end_point_quat_max = rad2deg(dist(end_point_quat,end_point_quat(1,:)));

% Home postion
x_h = T.x_end_avg(1).*1000; y_h = T.y_end_avg(1).*1000; z_h = T.z_end_avg(1).*1000;

% slice for manual inspection
sl = end_point_quat_max<5;

% create plot
fig = figure(1); clf; hold on; grid on;

x_w = 1000.*T.x_end_avg(sl);
y_w = 1000.*T.z_end_avg(sl);
z_w = 1000.*T.y_end_avg(sl);
x_w = x_w-x_w(1,:);
y_w = y_w-y_w(1,:); 
z_w = z_w-z_w(1,:); 

% conv-hull surface
[k1,av1] = convhull(x_w,y_w,z_w,'Simplify',true);
trisurf(k1,x_w,y_w,z_w,'FaceColor',"k",'EdgeAlpha',.3,'FaceAlpha',.1,'LineWidth',1.25)

% scatter of points
s = scatter3(x_w,y_w,z_w,'filled','MarkerFaceColor',map(3,:),'MarkerFaceAlpha',1,'MarkerEdgeColor','k','MarkerEdgeAlpha',1,'LineWidth',1.25);
s.SizeData = 150;

% change view
el = 0;
view(0,el)

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.75;
height = 0.75;
if el == 90
    height = 1.75;
end
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xlim([-110,110])
ylim([-110,110])
zlim([-20,60])

% export fig
if export_fig
    if el == 90
        exportgraphics(gcf,'../figures/workspace/x-y.png','Resolution',300*fig_s)
    end

    if el == 0
        exportgraphics(gcf,'../figures/workspace/z-x.png','Resolution',300*fig_s)
    end
end


%% Load in data

T = readtable("../data/repetability/repeatability_07_26_2023");
n_points = 100;

% find distance from mean, averge distance error, and distance variance
end_point = 1000.*[reshape(T.x_end_avg,n_points,1,[]),reshape(T.z_end_avg,n_points ,1,[]),reshape(T.y_end_avg,n_points ,1,[])];
end_point_mean = mean(end_point,3);
end_point_d = sqrt(sum((end_point-end_point_mean).^2,2));
e_point_d = abs((end_point-end_point_mean));
end_point_mean_e = mean(end_point_d,3);

% quaternion distance from mean
quat = [reshape(T.qx_end_avg,n_points,1,[]),reshape(T.qy_end_avg,n_points ,1,[]),reshape(T.qz_end_avg,n_points ,1,[]),reshape(T.qw_end_avg,n_points ,1,[])];
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
end_point_quat_mean_e = mean(end_point_quat_d,2);


%% Figure 2.1

fig = figure(21); clf; hold on; grid on

% Plot data
s = scatter3(end_point_mean(:,1)-mean(end_point_mean(:,1)),end_point_mean(:,2)-mean(end_point_mean(:,2)),end_point_mean(:,3)-mean(end_point_mean(:,3)),[],end_point_mean_e,'filled','MarkerEdgeColor',"k",'LineWidth',1.25);
s.SizeData = 150;
plot3(end_point_mean(:,1)-mean(end_point_mean(:,1)),end_point_mean(:,2)-mean(end_point_mean(:,2)),end_point_mean(:,3)-mean(end_point_mean(:,3)),'Color',[0, 0, 0, 0.5],'LineWidth',2)

% Set colormap and add color bar
colormap(gca,flipud(brewermap([],'Spectral')));
cb = colorbar('northoutside','LineWidth',1.5,'FontSize',ax_font_size);

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1.25;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);
view([45,37.5])

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);

% export fig
if export_fig
    exportgraphics(gcf,'../figures/repetability/pos_error.png','Resolution',300*fig_s)
end


%% Figure 2.2

fig = figure(22); clf; hold on; grid on

% Plot data
s = scatter3(end_point_mean(:,1)-mean(end_point_mean(:,1)),end_point_mean(:,2)-mean(end_point_mean(:,2)),end_point_mean(:,3)-mean(end_point_mean(:,3)),[],end_point_quat_mean_e,'filled','MarkerEdgeColor',"k",'LineWidth',1.25);
s.SizeData = 150;
plot3(end_point_mean(:,1)-mean(end_point_mean(:,1)),end_point_mean(:,2)-mean(end_point_mean(:,2)),end_point_mean(:,3)-mean(end_point_mean(:,3)),'Color',[0, 0, 0, 0.5],'LineWidth',2)

% Set colormap and add color bar
colormap(gca,flipud(brewermap([],'RdBu')));
cb = colorbar('northoutside','LineWidth',1.5,'FontSize',ax_font_size);

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1.25;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);
view([45,37.5])

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);

% export fig
if export_fig
    exportgraphics(gcf,'../figures/repetability/quat_error.png','Resolution',300*fig_s)
end


%% Figure 2.3

fig = figure(23); clf; hold on;

end_point_d_flat = reshape(end_point_d,1,[]);
histogram(end_point_d_flat,'facecolor',map(1,:),'facealpha',.2,'EdgeColor','none')
histogram(end_point_d_flat,'EdgeColor',map(1,:),'linewidth',3,'DisplayStyle','stairs')
mean(end_point_d_flat)

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1.25;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xlim([0,8])

% export fig
if export_fig
    exportgraphics(gcf,'../figures/repetability/pos_hist.png','Resolution',300*fig_s)
end


%% Figure 2.4

fig = figure(24); clf; hold on;

histogram(end_point_quat_d,'facecolor',map(2,:),'facealpha',.2,'EdgeColor','none')
histogram(end_point_quat_d,'EdgeColor',map(2,:),'linewidth',3,'DisplayStyle','stairs')
mean(reshape(end_point_quat_d,1,[]))

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1.25;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xlim([0,1.5])


% export fig
if export_fig
    exportgraphics(gcf,'../figures/repetability/quat_hist.png','Resolution',300*fig_s)
end

%%  Figure 3.1

T_f = load('../data/subset/15000_pose_data_07_26_2023.mat');
f_data = T_f.Y_test_forward;
f_metric = load('../metrics/forward_2023_09_18-19_38_06_5_883mm.mat');

fig = figure(31); clf; hold on; grid on

s=scatter3(f_data(:,1)-x_h,f_data(:,3)-z_h,f_data(:,2)-y_h,[],f_metric.test_error,'filled','MarkerEdgeColor',"k",'LineWidth',1.25);
s.SizeData = 150;

% Set colormap and add color bar
colormap(gca,flipud(brewermap([],'Spectral')));
cb = colorbar('northoutside','LineWidth',1.5,'FontSize',ax_font_size);

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1.25;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);
view([45,45])
xlim([-200,200])
ylim([-200,200])
xticks([-200,0,200])
yticks([-200,0,200])
xtickangle(0)
ytickangle(0)

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);


% export fig
if export_fig
    exportgraphics(gcf,'../figures/forward/pos_error.png','Resolution',300*fig_s)
end

%% Figure 3.2
fig = figure(32); clf; hold on;

histogram(f_metric.test_error,'facecolor',map(1,:),'facealpha',.2,'EdgeColor','none')
histogram(f_metric.test_error,'EdgeColor',map(1,:),'linewidth',3,'DisplayStyle','stairs')

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1.25;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);


% export fig
if export_fig
    exportgraphics(gcf,'../figures/forward/pos_hist.png','Resolution',300*fig_s)
end

%% Figure 3.3
fig = figure(33); clf; hold on; grid on

% Plot data
s = scatter3(f_data(:,1)-x_h,f_data(:,3)-z_h,f_data(:,2)-y_h,[],f_metric.test_q_error,'filled','MarkerEdgeColor',"k",'LineWidth',1.25);
s.SizeData = 150;

% Set colormap and add color bar
colormap(gca,flipud(brewermap([],'RdBu')));
cb = colorbar('northoutside','LineWidth',1.5,'FontSize',ax_font_size);

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1.25;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);
view([45,45])
xlim([-200,200])
ylim([-200,200])
xticks([-200,0,200])
yticks([-200,0,200])
xtickangle(0)
ytickangle(0)

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);

% export fig
if export_fig
    exportgraphics(gcf,'../figures/forward/quat_error.png','Resolution',300*fig_s)
end

%% Figure 3.4
fig = figure(34); clf; hold on;

histogram(f_metric.test_q_error,'facecolor',map(2,:),'facealpha',.2,'EdgeColor','none')
histogram(f_metric.test_q_error,'EdgeColor',map(2,:),'linewidth',3,'DisplayStyle','stairs')

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1.25;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);

% export fig
if export_fig
    exportgraphics(gcf,'../figures/forward/quat_hist.png','Resolution',300*fig_s)
end

%% Figure 4.1
fig = figure(41); clf; hold on; grid on
    
i_data = T_f.X_test_inverse;
i_metric = load('../metrics/inverse_2023_09_19-01_01_05_4_070pwm.mat');

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1.25;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);
view([45,45])
xlim([-200,200])
ylim([-200,200])
xticks([-200,0,200])
yticks([-200,0,200])
xtickangle(0)
ytickangle(0)

cmap = colormap(gca,flipud(brewermap([],'Spectral')));
clim([min(i_metric.test_error),max(i_metric.test_error)]); % set the colormap limits
cb = colorbar('northoutside','LineWidth',1.5,'FontSize',ax_font_size);
clim([min(i_metric.test_error),max(i_metric.test_error)]);

norm_error = (i_metric.test_error - min(i_metric.test_error)) / ...
             (max(i_metric.test_error) - min(i_metric.test_error));
color_indices = round(1 + norm_error * (size(cmap,1)-1));

for i = 1:length(cmap)
    s=scatter3(i_data(i,:,1)-x_h,i_data(i,:,3)-z_h,i_data(i,:,2)-y_h,[],cmap(color_indices(i),:),'filled','LineWidth',1.25,'Marker','o','MarkerEdgeColor',[0,0,0]);
    plot3(i_data(i,:,1)-x_h,i_data(i,:,3)-z_h,i_data(i,:,2)-y_h,'Color',[0, 0, 0, 0.5],'LineWidth',2)
    % set(s, 'MarkerEdgeAlpha', 0.5, 'MarkerFaceAlpha', 0.5)
    s.SizeData = 100;   
end

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);

% export fig
if export_fig
    exportgraphics(gcf,'../figures/inverse/error.png','Resolution',300*fig_s)
end

%% Figure 4.2
fig = figure(42); clf; hold on;

histogram(i_metric.test_error,'facecolor',map(1,:),'facealpha',.2,'EdgeColor','none')
histogram(i_metric.test_error,'EdgeColor',map(1,:),'linewidth',3,'DisplayStyle','stairs')

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1.25;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);

% export fig
if export_fig
    exportgraphics(gcf,'../figures/inverse/error_hist.png','Resolution',300*fig_s)
end

%% Figure 4.3
fig = figure(43); clf; hold on;

plot(T_f.)