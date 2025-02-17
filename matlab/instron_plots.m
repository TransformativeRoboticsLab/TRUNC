%% Global setup

close all; clear all; clc
warning('off','all')

% fig sizes and scale factor
fig_w = 300; fig_h = 300; fig_s = 3;

% fonts
ax_font_size = 5*fig_s;
legend_font_size = 5*fig_s;
set(0,'DefaultTextFontname', 'CMU Sans Serif' )
set(0,'DefaultAxesFontName', 'CMU Sans Serif' )

% colors
map = brewermap(9,'Set1');

% save path
export_fig = false;

% Geometric properties
r_t = 55/2; % size of truss sphere
r_s = 85/2; % size of simple sphere


%% Load-in all data

T_flex = readtable("./instron/force-displacement/flex-shaft.csv");

T_spf = readtable("./instron/force-displacement/simple-printed-force.csv");
T_spt_a = readtable("./instron/bending-rotation/simple-printed-bending.csv");
T_spt_p = readtable("./instron/torsion-rotation/simple-printed-torsion.csv");

T_ssf = readtable("./instron/force-displacement/simple-force-displacement.csv");
T_ssb = readtable("./instron/bending-rotation/simple-bending-rotation_1.csv");
T_sst = readtable("./instron/torsion-rotation/simple-torsion-rotation_1.csv");

T_tsf = readtable("./instron/force-displacement/complex-force-displacement.csv");
T_tsb = readtable("./instron/bending-rotation/complex-bending-rotation_1.csv");
T_tst = readtable("./instron/torsion-rotation/complex-torsion-rotation_1.csv");

%% Flex shaft plot

fig = figure(1); clf; hold on; box on;
grid on

plot(T_flex.Strain_0*10,T_flex.Stifness_0,'LineWidth',3,'LineStyle',':','color',"#F5A5A7")
errorbar(T_flex.Strain_0*10,T_flex.Stifness_0,T_flex.Stifness_1-T_flex.Stifness_0,T_flex.Stifness_2-T_flex.Stifness_0,'LineWidth',3,'LineStyle','none','color',map(1,:))


% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 2.25;
height = 1.75;
set(fig, 'Position', [0, 0, width*2, height*2]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
ylim([0,65])
xlim([-2.5,22])


% export fig
if export_fig
    exportgraphics(gcf,'../figures/instron/flex-shaft.png','Resolution',300*fig_s)
end

%% Simple printed force

fig = figure(2); clf; hold on;  box on;
grid on

shadedErrorBar(T_spf.Displacement_0,T_spf.Force_0,[T_spf.Force_2-T_spf.Force_0,-(T_spf.Force_1-T_spf.Force_0)],'lineProps',{'Color',map(3,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.5;
height = 1.25;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xticks([0,2,4,6,8])
yticks([0,2,4,6])
xlim([0,8])

% export fig
if export_fig
    exportgraphics(gcf,'../figures/instron/spf.png','Resolution',300*fig_s)
end

%% Simple printed torque

fig = figure(3); clf; hold on; box on;
grid on

shadedErrorBar(T_spt_a.Rotation_0,T_spt_a.Torque_0,[T_spt_a.Torque_2-T_spt_a.Torque_0,-(T_spt_a.Torque_1-T_spt_a.Torque_0)],'lineProps',{'Color',map(2,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})
shadedErrorBar(T_spt_p.Rotation_0,T_spt_p.Torque_0,[T_spt_p.Torque_2-T_spt_p.Torque_0,-(T_spt_p.Torque_1-T_spt_p.Torque_0)],'lineProps',{'Color',map(1,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.5;
height = 1.25;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xticks([0,0.1,.2,.3])
yticks([0,10,20,30,40])
xlim([0,.35])


% legend formatting
lg = legend('interpreter','latex','Location','northwest');
lg.FontSize = legend_font_size;
set(lg,'Box','off')
lg.ItemTokenSize(1) = 20;

% export fig
if export_fig
    exportgraphics(gcf,'../figures/instron/spt.png','Resolution',300*fig_s)
end

P = polyfit(deg2rad(mean(T_spt_a.Rotation_0,3)),mean(T_spt_a.Torque_0,3),1);
K_b_sp = P(1);
fprintf('\n Simple printed Kb is: %.2f (N-mm/rad) \n', K_b_sp);

P = polyfit(deg2rad(mean(T_spt_p.Rotation_0,3)),mean(T_spt_p.Torque_0,3),1);
K_t_sp = P(1);
fprintf('\n Simple printed Kt is: %.2f (N-mm/rad) \n', K_t_sp);

fprintf('\n Simple printed twist-to-bend is: %.2f \n', K_t_sp/K_b_sp);

%% Simple steel force

fig = figure(4); clf; hold on; box on;
grid on

T_ssf_Displacement = reshape(T_ssf.Displacement,76,1,[]);
T_ssf_Force = reshape(T_ssf.Force,76,1,[]);
T_ssf_Force_max = max(T_ssf_Force,[],3)-mean(T_ssf_Force,3);
T_ssf_Force_min = min(T_ssf_Force,[],3)-mean(T_ssf_Force,3);

P = polyfit(mean(T_ssf_Displacement,3),mean(T_ssf_Force,3),1);
K_a_ss = P(1);
fprintf('\n Simple spring steel Ka is: %.2f (N/mm) \n', K_a_ss);

shadedErrorBar(mean(T_ssf_Displacement,3),mean(T_ssf_Force,3),[reshape(T_ssf_Force_max,[],1),-reshape(T_ssf_Force_min,[],1)],'lineProps',{'Color',map(3,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xticks([0,5,10,15])
yticks([0,1,2,3,4])
ylim([0,4])
xlim([0,15.1])


% export fig
if export_fig
    exportgraphics(gcf,'../figures/instron/ssf.png','Resolution',300*fig_s)
end


%% Simple steel bending and torque

fig = figure(5); clf; hold on; box on;
grid on

T_ssb_Rotation = reshape(T_ssb.Rotation,199,1,[]);
T_ssb_Torque = reshape(T_ssb.Torque,199,1,[]).*1000;
T_ssb_Torque = T_ssb_Torque-T_ssb_Torque(1,:,:);
T_ssb_Torque_max = max(T_ssb_Torque,[],3)-mean(T_ssb_Torque,3);
T_ssb_Torque_min = min(T_ssb_Torque,[],3)-mean(T_ssb_Torque,3);

shadedErrorBar(deg2rad(mean(T_ssb_Rotation,3)),mean(T_ssb_Torque,3),[reshape(T_ssb_Torque_max,[],1),-reshape(T_ssb_Torque_min,[],1)],'lineProps',{'Color',map(2,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})

T_sst_Torque = -reshape(T_sst.Torque,199,1,[]).*1000;
T_sst_Torque = T_sst_Torque-T_sst_Torque(1,:,:);
T_sst_Torque_max = max(T_sst_Torque,[],3)-mean(T_sst_Torque,3);
T_sst_Torque_min = min(T_sst_Torque,[],3)-mean(T_sst_Torque,3);

shadedErrorBar(deg2rad(mean(T_ssb_Rotation,3)),mean(T_sst_Torque,3),[reshape(T_sst_Torque_max,[],1),-reshape(T_sst_Torque_min,[],1)],'lineProps',{'Color',map(1,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})

P = polyfit(deg2rad(mean(T_ssb_Rotation,3)),mean(T_ssb_Torque,3),1);
K_b_ss = P(1);
fprintf('\n Simple spring steel Kb is: %.2f (N-mm/rad) \n', K_b_ss);

F = (mean(T_ssb_Torque(end,:,:),3)/r_s);
L = 2*r_s;
y = r_s*sin(deg2rad(mean(T_ssb_Rotation(end,:,:),3)));
EI_ss = (F*L^3)/(3*y);
fprintf('\n Simple spring steel log EI/r^4 is: %.2f (N-m^2) \n', log10(EI_ss/1e6/((r_s/1000)^4)));


P = polyfit(deg2rad(mean(T_ssb_Rotation,3)),mean(T_sst_Torque,3),1);
K_t_ss = P(1);
fprintf('\n Simple spring steel Kt is: %.2f (N-mm/rad) \n', K_t_ss);
Fd = mean(T_sst_Torque(end,:,:),3);
theta = deg2rad(mean(T_ssb_Rotation(end,:,:),3));
L = 2*r_s;
GJ_ss = Fd/(theta/L);
fprintf('\n Simple spring steel log GJ/r^4 is: %.2f (N-m^2) \n', log10(GJ_ss/1e6/((r_s/1000)^4)));

fprintf('\n Simple spring steel twist-to-bend is: %.2f \n', K_t_ss/K_b_ss);

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xticks([0,0.1,.2,.3])
yticks([0,150,300,450])
xlim([0,.35])
ylim([0,500])


% legend formatting
lg = legend('interpreter','latex','Location','northwest');
lg.FontSize = legend_font_size;
set(lg,'Box','off')
lg.ItemTokenSize(1) = 20;

% export fig
if export_fig
    exportgraphics(gcf,'../figures/instron/sst.png','Resolution',300*fig_s)
end

%% Truss steel force

fig = figure(6); clf; hold on; box on;
grid on

T_tsf_Displacement = reshape(T_tsf.Displacement,76,1,[]);
T_tsf_Force = reshape(T_tsf.Force,76,1,[]);
T_tsf_Force_max = max(T_tsf_Force,[],3)-mean(T_tsf_Force,3);
T_tsf_Force_min = min(T_tsf_Force,[],3)-mean(T_tsf_Force,3);

shadedErrorBar(mean(T_tsf_Displacement,3),mean(T_tsf_Force,3),[reshape(T_tsf_Force_max,[],1),-reshape(T_tsf_Force_min,[],1)],'lineProps',{'Color',map(3,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})

P = polyfit(mean(T_tsf_Displacement,3),mean(T_tsf_Force,3),1);
K_a_ts = P(1);
fprintf('\n Truss spring steel Ka is: %.2f (N/mm) \n', K_a_ts);

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xticks([0,5,10,15])
yticks([0,1,2,3,4])
ylim([0,4])
xlim([0,15.1])


% export fig
if export_fig
    exportgraphics(gcf,'../figures/instron/tsf.png','Resolution',300*fig_s)
end

%% Truss steel torque

fig = figure(7); clf; hold on; box on;
grid on

T_tsb_Rotation = reshape(T_tsb.Rotation,199,1,[]);
T_tsb_Torque = reshape(T_tsb.Torque,199,1,[]).*1000;
T_tsb_Torque = T_tsb_Torque-T_tsb_Torque(1,:,:);
T_tsb_Torque_max = max(T_tsb_Torque,[],3)-mean(T_tsb_Torque,3);
T_tsb_Torque_min = min(T_tsb_Torque,[],3)-mean(T_tsb_Torque,3);

shadedErrorBar(deg2rad(mean(T_tsb_Rotation,3)),mean(T_tsb_Torque,3),[reshape(T_tsb_Torque_max,[],1),-reshape(T_tsb_Torque_min,[],1)],'lineProps',{'Color',map(2,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})

T_tst_Torque = -reshape(T_tst.Torque,199,1,[]).*1000;
T_tst_Torque = T_tst_Torque-T_tst_Torque(1,:,:);
T_tst_Torque_max = max(T_tst_Torque,[],3)-mean(T_tst_Torque,3);
T_tst_Torque_min = min(T_tst_Torque,[],3)-mean(T_tst_Torque,3);

shadedErrorBar(deg2rad(mean(T_tsb_Rotation,3)),mean(T_tst_Torque,3),[reshape(T_tst_Torque_max,[],1),-reshape(T_tst_Torque_min,[],1)],'lineProps',{'Color',map(1,:),'lineWidth',3,'MarkerSize',10,"DisplayName",""})

P = polyfit(deg2rad(mean(T_tsb_Rotation,3)),mean(T_tsb_Torque,3),1);
K_b_ts = P(1);
fprintf('\n Complex spring steel Kb is: %.2f (N-mm/rad) \n', K_b_ts);

F = (mean(T_tsb_Torque(end,:,:),3)/r_t);
L = 2*r_t;
y = r_t*sin(deg2rad(mean(T_tsb_Rotation(end,:,:),3)));
EI_ts = (F*L^3)/(3*y);
EI_ts_norm = log10(EI_ts/1e6/((r_t/1000)^4));
fprintf('\n Truss spring steel log EI/r^4 is: %.2f (N-m^2) \n', EI_ts_norm);

P = polyfit(deg2rad(mean(T_tsb_Rotation,3)),mean(T_tst_Torque,3),1);
K_t_ts = P(1);
fprintf('\n Complex spring steel Kt is: %.2f (N-mm/rad) \n', K_t_ts);

Fd = mean(T_tst_Torque(end,:,:),3);
theta = deg2rad(mean(T_tsb_Rotation(end,:,:),3));
L = 2*r_t;
GJ_ts = Fd/(theta/L);
GJ_ts_norm = log10(GJ_ts/1e6/((r_t/1000)^4));
fprintf('\n Truss spring steel log GJ/r^4 is: %.2f (N-m^2) \n', GJ_ts_norm);

fprintf('\n Complex spring steel twist-to-bend is: %.2f \n', K_t_ts/K_b_ts);

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xticks([0,0.1,.2,.3])
yticks([0,150,300,450])
xlim([0,.35])
ylim([0,500])

% legend formatting
lg = legend('interpreter','latex','Location','northwest');
lg.FontSize = legend_font_size;
set(lg,'Box','off')
lg.ItemTokenSize(1) = 20;

% export fig
if export_fig
    exportgraphics(gcf,'../figures/instron/tst.png','Resolution',300*fig_s)
end

%% EI vs GJ comparison of bio materials


% Read data from CSV file
data = readtable('./appendix_table.csv');

% Define groups and corresponding markers
groups = unique(data.Group);
markers = {'d', 's', 'o', '^', 'p', '+', '>', '*', 'h'};

% Logarithmic transformation for EI/r^4
data.logEI = log10(data.EI_Nm2_ ./ ((data.Diameter_m_/2) .^ 4));
data.logGJ = log10(data.GJ_Nm2_ ./ ((data.Diameter_m_/2) .^ 4));

% Create scatter plot
figure;
hold on;
for i = 1:length(groups)
    group_data = data(strcmp(data.Group, groups{i}), :);
    scatter(group_data.logGJ, group_data.logEI, 'filled', markers{i});
end

scatter(GJ_ts_norm,EI_ts_norm)

% Set axis labels
xlabel('log GJ / r^4');
ylabel('log EI / r^4');

% Add legend
legend(groups, 'Location', 'best');

% Set axis limits to match the example
xlim([4 12]);
ylim([4 12]);

% Display grid
grid on;
hold off;
