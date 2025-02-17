%% Global setup

close all; clear all; clc
warning('off','all')

% fig sizes and scale factor
fig_w = 300; fig_h = 300; fig_s = 3;

% fonts
ax_font_size = 6*fig_s;
legend_font_size = 4*fig_s;
set(0,'DefaultTextFontname', 'CMU Sans Serif' )
set(0,'DefaultAxesFontName', 'CMU Sans Serif' )

% colors
map = brewermap(9,'Set1');

% save path
export_fig = true;

%% Steel

steel = readtable("./instron/bellows/steel-bellows_1.csv");

plot(steel.Rotation)

pks = find(islocalmin(steel.Rotation));
pks = [pks;length(steel.Rotation)];
offset = 150;
offset_end = -5;

figure(1); clf; hold on;

Kt = zeros(1,5);

prev_pk = 1;
for i = 1:length(pks)
    pk = pks(i);
    tq = steel.Torque(prev_pk+offset:pk+offset_end);
    rt = steel.Rotation(prev_pk+offset:pk+offset_end);
    prev_pk = pk;
    plot(deg2rad(rt),tq)
    Kt(i) = abs(tq(1)-tq(end))/abs(deg2rad(rt(1)-rt(end)));
end

steel_kt = mean(Kt);
steel_K = Kt;


disp(mean(Kt))

%% Rubber

rubber = readtable("./instron/bellows/rubber-bellows_1.csv");

pks = find(islocalmin(rubber.Rotation));
pks = [pks;length(rubber.Rotation)];
offset = 150;
offset_end = -5;

figure(2); clf; hold on;

Kt = zeros(1,5);

prev_pk = 1;
for i = 1:length(pks)
    pk = pks(i);
    tq = rubber.Torque(prev_pk+offset:pk+offset_end);
    rt = rubber.Rotation(prev_pk+offset:pk+offset_end);
    prev_pk = pk;
    plot(deg2rad(rt),tq)
    Kt(i) = abs(tq(end)-tq(1))/abs(deg2rad(rt(1)-rt(end)));
end

rubber_kt = mean(Kt);
rubber_K = Kt;

disp(mean(Kt))

%% Truss 

truss_kt = 1499.87;
truss_K = [1.4961e+03,1.5043e+03];

fig = figure(1); clf; hold on; grid on; box on;

bar(0,steel_kt,'FaceColor',"#FFBB00",'FaceAlpha',1);
% errorbar(0,steel_kt,min(steel_K)-steel_kt,max(steel_K)-steel_kt,'LineWidth',1.5,'Color','k');    


bar(1,rubber_kt,'FaceColor',"k",'FaceAlpha',1);
% errorbar(1,rubber_kt,min(rubber_K)-rubber_kt,max(rubber_K)-rubber_kt,'LineWidth',1.5,'Color','k');    


bar(2,truss_kt,'FaceColor',map(2,:),'FaceAlpha',1);
% errorbar(2,truss_kt,min(truss_K)-truss_kt,max(truss_K)-truss_kt,'LineWidth',1.5,'Color','k');    



% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 2;
height = 1.75;
set(fig, 'Position', [0, 0, width*2.5, height*2.5]);

xlim([-.5,2.5])
xticklabels([]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);

ax = gca;

ax.YAxis.Exponent = 3;

% export fig
if export_fig
    exportgraphics(gcf,'../figures/efficiency/stiffness-plot.png','Resolution',300*fig_s)
end


%% Nesting analysis

trunc = (26.675^3)/(28.125^3);
steel_bellow = ((pi/4)*4.7625^2)/((pi/4)*9.525^2);
rubber_bellow = ((pi/4)*15.875^2)/((pi/4)*26.9875^2);

fig = figure(2); clf; hold on; grid on; box on;

bar(0,100*steel_bellow,'FaceColor',"#FFBB00",'FaceAlpha',1);

bar(1,100*rubber_bellow,'FaceColor',"k",'FaceAlpha',1);

bar(2,100*trunc,'FaceColor',map(2,:),'FaceAlpha',1);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 2;
height = 1.75;
set(fig, 'Position', [0, 0, width*2.5, height*2.5]);

xlim([-.5,2.5])
xticklabels([]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);

ax = gca;

% export fig
if export_fig
    exportgraphics(gcf,'../figures/efficiency/nesting-plot.png','Resolution',300*fig_s)
end
